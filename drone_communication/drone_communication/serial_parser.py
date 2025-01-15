import serial
import struct
from dataclasses import dataclass
import rclpy
import rclpy.logging
from rclpy.node import Node
from drone_communication_interface.msg import ByteArray
from std_msgs.msg import String

BUF_SIZE = 1490


@dataclass
class Message:
    agent_id: int
    winning_bids: list
    winning_agents: list


def byte_array_to_message(data):
    agent_id = struct.unpack("B", data[:1])[0]
    data = data[1:]
    n_bids = len(data) // 5
    bids_data = data[: n_bids * 4]
    agents_data = data[n_bids * 4 :]
    winning_bids = list(struct.unpack(f"{n_bids}f", bids_data))
    winning_agents = list(struct.unpack(f"{n_bids}B", agents_data))
    return Message(agent_id, winning_bids, winning_agents)


def message_to_byte_array(message: Message):
    agent_id_data = struct.pack("B", message.agent_id)
    bids_data = struct.pack(f"{len(message.winning_bids)}f", *message.winning_bids)
    agents_data = struct.pack(
        f"{len(message.winning_agents)}B", *message.winning_agents
    )
    return agent_id_data + bids_data + agents_data


class SerialParser(Node):

    def __init__(self, port):
        super().__init__("serial_parser")
        self.port = port
        self.serial_read_buffer = bytearray()
        self.ser = serial.Serial(port, timeout=0.1)
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()
        self.publisher_ = self.create_publisher(String, "message", 10)
        self.message_broadcast_subscriber = self.create_subscription(
            ByteArray, "broadcast", self.send_message_callback, 10
        )

        # Define the serial receive timer
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.serial_read_callback)

        # internal variables
        self.start_sequence = b"\x02\x02\x02"
        self.stop_sequence = b"\x03\x03\x03"

    def serial_read_callback(self):
        received_message = None
        try:
            # self.get_logger().info("Reading from serial")
            received_message = self.receive_message_from_serial()
            if received_message:
                self.get_logger().info(
                    f"Received message from agent id: {received_message.agent_id}"
                )
        except Exception as e:
            self.get_logger().error(f"Failed to receive message: {e}")

    def send_message_callback(self, msg: ByteArray):
        self.get_logger().info(f"Sending message to serial with size: {len(msg.bytes)}")
        try:
            self.send_message_to_serial(self.port, msg.bytes)
        except Exception as e:
            self.get_logger().error(f"Failed to send message: {e}")

    def send_message_to_serial(self, port, message: Message):
        data = message_to_byte_array(message)
        if len(data) > 1490:
            raise ValueError("Data size exceeds 1490 bytes")
        with serial.Serial(port) as ser:
            ser.write(data)
            ser.flush()
    
    def receive_message_from_serial(self):
        self.serial_read_buffer.extend(self.ser.read(self.ser.in_waiting))
        # buffer.extend(ser.read(ser.in_waiting))
        start_index = self.serial_read_buffer.find(self.start_sequence)
        stop_index = self.serial_read_buffer.find(self.stop_sequence)
        
        if stop_index < start_index:
            self.serial_read_buffer = self.serial_read_buffer[start_index:]
        else: 
            if start_index < stop_index and (stop_index - start_index) < BUF_SIZE:
                data = self.serial_read_buffer[start_index + len(self.start_sequence) : stop_index]
                self.serial_read_buffer = self.serial_read_buffer[stop_index + len(self.stop_sequence) :]
                try: 
                    message = byte_array_to_message(data)
                    self.get_logger().info(f"Received message: {message}")
                    self.publisher_.publish(String(data=str(message)))
                except struct.error as e:
                    self.get_logger().error(f"Failed to deserialize message: {e}")


def main():
    rclpy.init()
    serial_parser = SerialParser("/dev/ttyACM0")
    rclpy.spin(serial_parser)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
