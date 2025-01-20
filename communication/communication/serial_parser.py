import serial
import struct
from dataclasses import dataclass
import rclpy
import rclpy.logging
from rclpy.node import Node
from common_interface.msg import ByteArray
from std_msgs.msg import String
import random
import threading
import time
BUF_SIZE = 1490

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
    def deserialize(data: bytes, unescape_message: bool = False) -> 'Message':
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
        escaped_data.append(start_delimiter)  # Add start byte
        for byte in data:
            if byte in (start_delimiter, end_delimiter, escape_byte):
                escaped_data.append(escape_byte)  # add escape byte
            escaped_data.append(byte)
        escaped_data.append(end_delimiter)  # Add end byte
        return bytes(escaped_data)

    @staticmethod
    def remove_escape_from_message(data):
        start_delimiter = b'\x02'
        end_delimiter = b'\x03'
        escape_char = b'\x1B'

        data = data.replace(escape_char + start_delimiter, start_delimiter)
        data = data.replace(escape_char + end_delimiter, end_delimiter)
        data = data.replace(escape_char + escape_char, escape_char)

        if data[0] == start_delimiter[0]:
            data = data[1:]
        if data[-1] == end_delimiter[0]:
            data = data[:-1]
        return data


class SerialParser(Node):

    def __init__(self, port):
        super().__init__("serial_parser")
        self.port = port
        self.serial_read_buffer = bytearray()
        
        # Open serial and flush
        self.ser = serial.Serial(port)
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()
        self.ser.flush()
        time.sleep(0.1)
        
        self.publisher_ = self.create_publisher(String, "message", 10)
        self.message_broadcast_subscriber = self.create_subscription(
            ByteArray, "broadcast", self.send_message_callback, 10
        )

        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.serial_read_callback)
        self.last_received_time = time.time()

    def serial_read_callback(self):
        try:
            received_message = None
            received_message = self.receive_message_from_serial()
            if received_message:
                self.get_logger().info(f"Received message from agent id: {received_message.agent_id}")
                self.publisher_.publish(String(data=str(received_message)))
        except Exception as e:
            self.get_logger().error(f"Failed to receive message: {e}")

    def send_message_callback(self, msg: ByteArray):
        self.get_logger().info(f"Sending message to serial with size: {len(msg.bytes)}")
        try:
            self.send_message_to_serial(self.port, msg.bytes)
        except Exception as e:
            self.get_logger().error(f"Failed to send message: {e}")

    def send_message_to_serial(self, port, message: Message):
        data = message.serialize()
        if len(data) > 1490:
            raise ValueError("Data size exceeds 1490 bytes")
        with serial.Serial(port) as ser:
            ser.write(data)
            ser.flush()

    def receive_message_from_serial(self):
        start_delimiter = b'\x02'
        end_delimiter = b'\x03'
        escape_char = b'\x1B'
        while True:
            byte_read = self.ser.read()
            if byte_read:
                if byte_read == start_delimiter and not self.serial_read_buffer:
                    self.serial_read_buffer.extend(byte_read)
                elif byte_read == escape_char:
                    self.serial_read_buffer.extend(self.ser.read())  # ignore escape char and read the next byte
                elif byte_read == end_delimiter:
                    self.serial_read_buffer.extend(byte_read)
                    start_index = self.serial_read_buffer.find(start_delimiter)
                    stop_index = len(self.serial_read_buffer) - 1
                    data = self.serial_read_buffer[start_index + 1: stop_index]
                    self.serial_read_buffer.clear()
                    try:
                        return Message.deserialize(data)
                    except struct.error as e:
                        self.get_logger().error(f"Failed to deserialize message: {e}")
                        hex_data = ' '.join(f'{byte:02x}' for byte in data)
                        self.get_logger().error(f"Not able to deserialize: {len(data)} Bytes")
                        for i in range(0, len(hex_data), 48):
                            self.get_logger().error(f" {hex_data[i:i+48]}")
                else:
                    self.serial_read_buffer.extend(byte_read)

def main():
    rclpy.init()
    serial_parser = SerialParser("/dev/ttyACM0")
    rclpy.spin(serial_parser)
    rclpy.shutdown()


def send_message_to_serial(port, message: Message):
    data = message.serialize()
    if len(data) > 1490:
        raise ValueError("Data size exceeds 1490 bytes")
    with serial.Serial(port) as ser:
        print(f"Sending message to serial with size: {len(data)}")
        ser.write(data)
        ser.flush()


def generate_random_floats(count, lower_bound, upper_bound):
    return [float(random.uniform(lower_bound, upper_bound)) for _ in range(count)]


def sender_thread():
    sender_port = "/dev/ttyACM1"  # Change this to your serial port
    lower_bound = 0
    upper_bound = 10
    # n_tasks = 295
    n_tasks = 10

    def send_periodically():
        while True:
            # time.sleep(0.01) # This is the minimum delay
            agent_id = random.randint(0, 255)
            winning_bids = generate_random_floats(n_tasks, lower_bound, upper_bound)
            timestamps = generate_random_floats(n_tasks, lower_bound, 1000)
            winning_agents = [random.randint(0, 255) for _ in range(n_tasks)]
            message = Message(agent_id, winning_bids, winning_agents,timestamps)
            send_message_to_serial(sender_port, message)

    send_thread = threading.Thread(target=send_periodically)
    send_thread.start()


if __name__ == "__main__":
    sender_thread()
    main()

