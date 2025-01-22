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
class Message:
    agent_id: int
    winning_bids: list
    winning_agents: list
    timestamps: list

    def serialize(self, escape_message: bool = True) -> bytes:
        agent_id_data = struct.pack("B", self.agent_id)
        bids_data = struct.pack(f"{len(self.winning_bids)}f", *self.winning_bids)
        agents_data = struct.pack(f"{len(self.winning_agents)}B", *self.winning_agents)
        timestamps_data = struct.pack(f"{len(self.timestamps)}f", *self.timestamps)
        data = agent_id_data + bids_data + agents_data + timestamps_data
        if escape_message:
            data = self.add_escape_to_message(data)
        return data

    @staticmethod
    def deserialize(data: bytes, unescape_message: bool = False) -> "Message":
        if unescape_message:
            data = Message.remove_escape_from_message(data)
        agent_id = struct.unpack("B", data[:1])[0]
        data = data[1:]
        n_bids = len(data) // 9
        bids_data = data[: n_bids * 4]
        agents_data = data[n_bids * 4 : n_bids * 5]
        timestamps_data = data[n_bids * 5 :]
        winning_bids = list(struct.unpack(f"{n_bids}f", bids_data))
        winning_agents = list(struct.unpack(f"{n_bids}B", agents_data))
        timestamps = list(struct.unpack(f"{n_bids}f", timestamps_data))
        return Message(agent_id, winning_bids, winning_agents, timestamps)

    @staticmethod
    def add_escape_to_message(data):
        start_delimiter = 0x02
        end_delimiter = 0x03
        escape_byte = 0x1B

        escaped_data = bytearray()
        escaped_data.append(start_delimiter)
        for byte in data:
            if byte in (start_delimiter, end_delimiter, escape_byte):
                escaped_data.append(escape_byte)
            escaped_data.append(byte)
        escaped_data.append(end_delimiter)
        return bytes(escaped_data)

    @staticmethod
    def remove_escape_from_message(data):
        start_delimiter = b"\x02"
        end_delimiter = b"\x03"
        escape_char = b"\x1B"

        data = data.replace(escape_char + start_delimiter, start_delimiter)
        data = data.replace(escape_char + end_delimiter, end_delimiter)
        data = data.replace(escape_char + escape_char, escape_char)

        if data[0] == start_delimiter[0]:
            data = data[1:]
        if data[-1] == end_delimiter[0]:
            data = data[:-1]
        return data

class PlannerNode(Node):
    def __init__(self, coverage_problem: CoverageProblem.CoverageProblem, initial_position):
        super().__init__("planner")
        
        self.declare_parameter('n_agents', 3)
        self.declare_parameter('agent_id', 0)
        self.declare_parameter('capacity', 3000)

        n_agents = self.get_parameter('n_agents').get_parameter_value().integer_value
        agent_id = self.get_parameter('agent_id').get_parameter_value().integer_value
        capacity = self.get_parameter('capacity').get_parameter_value().integer_value
        
        self.get_logger().info(f"Number of agents: {n_agents}")
        self.get_logger().info(f"Agent ID: {agent_id}")
        self.get_logger().info(f"Capacity: {capacity}")
        # qos_profile = rclpy.qos.QoSProfile(
        #     reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
        #     durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
        #     depth=10
        # )
        
        # Create a queue for incoming messages. Only the newest message from each agent is kept.
        self.incoming_queue = queue.Queue()
        self.outgoing_queue = queue.Queue()

        self.agent = CBBA.agent(
            id=agent_id,
            state=shapely.Point(initial_position),
            tasks=np.array(coverage_problem.getTasks()),
            capacity=capacity,
            number_of_agents=n_agents,
        )
        
        # Agent Communication
        self.message_subscriber = self.create_subscription(ByteArray, "/communication/message", self.message_callback, 10)
        self.message_broadcaster = self.create_publisher(ByteArray, "/communication/broadcast", 10)
        
        # The agent messages
        self.agent_messages = {}
        
        self.timer = self.create_timer(1, self.process_queues)
        self.get_logger().info("PlannerNode has started.")

    def message_callback(self, msg):
        try:
            self.get_logger().info(f"Received message with {len(msg.bytes)} bytes")
            message = Message.deserialize(bytes(b''.join(msg.bytes)))
            self.get_logger().info(f"Received message: {message}")
            self.incoming_queue.put(message)
        except Exception as e:
            self.get_logger().error(f"Failed to process message: {e}")

    def process_queues(self):
        messages = []
        while not self.incoming_queue.empty():
            messages.append(self.incoming_queue.get())

        self.consensus(messages)
        self.bundle_builder()
        
        # Only build a bundle if there is new information in the buffer or if the bundle is built for the first time
        if not self.incoming_queue.empty() or len(self.agent.bundle) == 0:
            self.bundle_builder()

    def consensus(self, messages):
        for message in messages:
            if message.agent_id not in self.agent_messages:
                self.agent_messages[message.agent_id] = [[],[],[]]
            self.agent_messages[message.agent_id][0] = message.winning_bids
            self.agent_messages[message.agent_id][1] =  message.winning_agents
            self.agent_messages[message.agent_id][2] = message.timestamps
            
        self.agent.receive_message(self.agent_messages)
        self.agent.update_task()
            
    def bundle_builder(self):
        # Build the bundle using the newest state from the listener
        bids: CBBA.BundleResult = self.agent.build_bundle() 
        self.get_logger().info(str(self.agent.bundle))
        bid_message= Message(self.agent.id,winning_bids=bids.winning_bids, winning_agents=bids.winning_agents, timestamps=bids.timestamps)
        data = bid_message.serialize()
        self.get_logger().info(f"Broadcast message {str(bid_message)}")
        self.get_logger().info(f"Broadcast message with {len(data)} bytes")
        self.get_logger().info(f"Broadcast byte message {data}")
        
        deserialized_message = Message.deserialize(data,unescape_message=True)
        self.get_logger().info(f"Deserialized message {str(deserialized_message)}")
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
