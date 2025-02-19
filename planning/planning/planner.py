import rclpy
from rclpy.node import Node
from common_interface.msg import ByteArray
from std_msgs.msg import Byte, ByteMultiArray
import struct
import random
import threading
import serial
from dataclasses import dataclass
import queue
from trajallocpy import Agent, CoverageProblem, Experiment, Task, Utility, CBBA
import shapely
import geojson
import numpy as np

@dataclass
class StatusMessage:
    agnet_id: int
    rssi: int

    def serialize(self) -> bytes:
        return struct.pack("ii", self.agnet_id, self.rssi)

    @staticmethod
    def deserialize(data: bytes) -> 'StatusMessage':
        agnet_id, rssi = struct.unpack("ii", data)
        return StatusMessage(agnet_id, rssi)


@dataclass
class Message:
    agent_id: int
    winning_bids: list
    winning_agents: list
    timestamps: list

    def serialize(self) -> bytes:
        agent_id_data = struct.pack("B", self.agent_id)
        bids_data = struct.pack(f"{len(self.winning_bids)}f", *self.winning_bids)
        agents_data = struct.pack(f"{len(self.winning_agents)}b", *self.winning_agents)
        timestamps_data = struct.pack(f"{len(self.timestamps)}f", *self.timestamps)
        data = agent_id_data + bids_data + agents_data + timestamps_data
        return data

    @staticmethod
    def deserialize(data: bytes, num_bids,num_timestamps, unescape_message: bool = False) -> 'Message':
        if unescape_message:
            data = Message.remove_escape_from_message(data)

        # Extract agent_id (1 byte)
        agent_id = struct.unpack("B", data[:1])[0]
        offset = 1

        # Extract winning_bids (num_bids * 4 bytes for each float)
        bids_format = f"{num_bids}f"
        bids_size = struct.calcsize(bids_format)
        winning_bids = struct.unpack(bids_format, data[offset:offset + bids_size])
        offset += bids_size

        # Extract winning_agents (num_agents * 1 byte for each agent)
        agents_format = f"{num_bids}b"
        agents_size = struct.calcsize(agents_format)
        winning_agents = struct.unpack(agents_format, data[offset:offset + agents_size])
        offset += agents_size

        # Extract timestamps (num_timestamps * 4 bytes for each float)
        timestamps_format = f"{num_timestamps}f"
        timestamps_size = struct.calcsize(timestamps_format)
        timestamps = struct.unpack(timestamps_format, data[offset:offset + timestamps_size])

        return Message(agent_id, winning_bids, winning_agents, timestamps)


class PlannerNode(Node):
    def __init__(self, coverage_problem: CoverageProblem.CoverageProblem, initial_position):
        super().__init__("planner")
        
        self.declare_parameter('n_agents', 2)
        self.declare_parameter('agent_id', 0)
        self.declare_parameter('capacity', 3000)

        self.n_agents = self.get_parameter('n_agents').get_parameter_value().integer_value
        self.agent_id = self.get_parameter('agent_id').get_parameter_value().integer_value
        self.capacity = self.get_parameter('capacity').get_parameter_value().integer_value
        
        self.get_logger().info(f"Number of agents: {self.n_agents}")
        self.get_logger().info(f"Agent ID: {self.agent_id}")
        self.get_logger().info(f"Capacity: {self.capacity}")
        # qos_profile = rclpy.qos.QoSProfile(
        #     reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
        #     durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
        #     depth=10
        # )
        
        # Create a queue for incoming messages. Only the newest message from each agent is kept.
        self.incoming_queue = queue.Queue()
        self.outgoing_queue = queue.Queue()

        self.agent = CBBA.agent(
            id=self.agent_id,
            state=shapely.Point(initial_position),
            tasks=np.array(coverage_problem.getTasks()),
            capacity=self.capacity,
            number_of_agents=self.n_agents,
        )
        
        # Agent Communication
        self.message_subscriber = self.create_subscription(ByteArray, "/communication/message", self.message_callback, 10)
        self.message_broadcaster = self.create_publisher(ByteArray, "/communication/broadcast", 10)
        
        # The agent messages
        self.agent_messages = {}
        
        self.timer = self.create_timer(1, self.process_queues)
        self.timer = self.create_timer(0.5, self.send_status_message)
        self.get_logger().info("PlannerNode has started.")

    def send_status_message(self):
        self.get_logger().info("Sending status message")
        data = StatusMessage(self.agent_id, random.randint(0, 0)).serialize()
        self.message_broadcaster.publish(ByteArray(bytes=[bytes([b]) for b in data]))

    def message_callback(self, msg):
        try:
            try:
                message = StatusMessage.deserialize(bytes(b''.join(msg.bytes)))
                self.get_logger().info(f"Received status message from agent {message.agnet_id} with RSSI {message.rssi}")
            except Exception as e:
                message = Message.deserialize(bytes(b''.join(msg.bytes)), num_bids=len(self.agent.tasks), num_timestamps=self.n_agents)
                self.get_logger().info(f"Received Bid message with {len(msg.bytes)} bytes")
                # self.get_logger().info(f"Received message: {message}")
                self.incoming_queue.put(message)
        except Exception as e:
            self.get_logger().error(f"Failed to process message: {e}")

    def process_queues(self):
        messages = []
        while not self.incoming_queue.empty():
            messages.append(self.incoming_queue.get())

        conflicts = self.consensus(messages)
        if conflicts != 0 or len(self.agent.bundle) == 0:
            self.bundle_builder()
        else:
            self.get_logger().info("No conflicts found, no need to update the bundle")

    def consensus(self, messages):
        for message in messages:
            if message.agent_id not in self.agent_messages:
                self.agent_messages[message.agent_id] = [[],[],[]]
            self.agent_messages[message.agent_id][0] = message.winning_bids
            self.agent_messages[message.agent_id][1] = message.winning_agents
            self.agent_messages[message.agent_id][2] = {agent_id: random.uniform(0, 1000)  for i, agent_id in enumerate(range(self.n_agents))}
            
        self.agent.receive_message(self.agent_messages)
        return self.agent.update_task()

            
    def bundle_builder(self):
        # Build the bundle using the newest state from the listener
        bids: CBBA.BundleResult = self.agent.build_bundle() 
        self.get_logger().info(str(self.agent.bundle))
        bid_message= Message(self.agent.id,winning_bids=list(bids.winning_bids), winning_agents=list(bids.winning_agents), timestamps=list(bids.timestamps.values()))
        self.get_logger().info(f"Serialized message: {bid_message}")
        data = bid_message.serialize()
        self.get_logger().info(f"Broadcast message with {len(data)} bytes")
        self.message_broadcaster.publish(ByteArray(bytes=[bytes([b]) for b in data]))


def load_coverage_problem() -> CoverageProblem.CoverageProblem:
    file_name = "planning/planning/environment.geojson"
    with open(file_name) as json_file:
        geojson_file = geojson.load(json_file)
        features = geojson_file["features"]

    geometries = {
        "obstacles": shapely.MultiPolygon(),
        "tasks": shapely.MultiLineString(),
        "boundary": shapely.Polygon(),
    }

    for feature in features:
        if feature["geometry"]:
            geometries[feature["id"]] = shapely.geometry.shape(feature["geometry"])
    number_of_tasks = len(list(geometries["tasks"].geoms))

    print(file_name, " Tasks: ", number_of_tasks)
    # Initialize coverage problem and the agents
    geometries["boundary"] = geometries["boundary"].buffer(1)

    # Scale each polygon in the MultiPolygon
    scaled_polygons = []
    for polygon in geometries["obstacles"].geoms:
        polygon: shapely.geometry.Polygon
        scaled_polygon = polygon.buffer(-1) 
        scaled_polygons.append(scaled_polygon)

    # Create a new MultiPolygon with scaled polygons
    scaled_multi_polygon = shapely.geometry.MultiPolygon(scaled_polygons)
    task_list = []
    for id, task in enumerate(geometries["tasks"].geoms):
        task_list.append(Task.TrajectoryTask(id, task, reward=1))

    cp = CoverageProblem.CoverageProblem(
        restricted_areas=scaled_multi_polygon,
        search_area=geometries["boundary"],
        tasks=task_list,
    )
    return cp


def main(args=None):
    
    rclpy.init(args=args)
    cp = load_coverage_problem()
    
    node = PlannerNode(cp,initial_position=cp.generate_random_point_in_problem())
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
