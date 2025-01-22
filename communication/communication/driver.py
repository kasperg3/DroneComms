import serial
import rclpy
import rclpy.logging
from rclpy.node import Node
from common_interface.msg import ByteArray
import time

    
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
class CommunicationDriver(Node):

    def __init__(self, port = "/dev/ttyACM0"):
        super().__init__("driver")
        self.declare_parameter("port", port)
        
        self.port = self.get_parameter("port").get_parameter_value().string_value
        if self.port is None:
            self.get_logger().error(f"Port parameter is not set, using default {port}")
            self.port = port
        self.serial_read_buffer = bytearray()
        
        # Open serial and flush
        self.ser = serial.Serial(port)
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()
        self.ser.flush()
        time.sleep(0.1)
        
        # Subscribe to the messages received from the serial port
        self.publisher_ = self.create_publisher(ByteArray, "/communication/message", 10)
        # Broad cast the messages to the serial port
        timer_period = 0.1  # seconds
        
        self.timer = self.create_timer(timer_period, self.serial_read_callback)
        self.last_received_time = time.time()
        self.message_broadcast_subscriber = self.create_subscription(ByteArray, "/communication/broadcast", self.send_message_callback, 10)
        self.get_logger().info("CommunicationDriver has started.")

    def serial_read_callback(self):
        try:
            received_message = None
            received_message = self.receive_message_from_serial()
            if received_message:
                self.get_logger().info(f"Received message with {len(received_message)} bytes")
                
                message = ByteArray(bytes=[bytes([b]) for b in received_message])
                self.get_logger().info(f"Sending message with length {len(message.bytes)}")
                self.publisher_.publish(message)
        except Exception as e:
            self.get_logger().error(f"Failed to receive message: {e}")

    def send_message_callback(self, msg: ByteArray):
        self.get_logger().info(f"Sending message with to serial port {self.port} with {len(msg.bytes)} bytes")
        try: 
            self.send_message_to_serial(self.port, bytes(b''.join(msg.bytes)))
        except Exception as e:
            self.get_logger().error(f"Failed to send message: {e}")

    def send_message_to_serial(self, port, data: bytearray):
        if len(data) > 1490:
            raise ValueError("Data size exceeds 1490 bytes")
        with serial.Serial(port) as ser:
            ser.write(add_escape_to_message(data))
            ser.flush()

    def receive_message_from_serial(self):
        start_delimiter = b'\x02'
        end_delimiter = b'\x03'
        escape_char = b'\x1B'
        while self.ser.in_waiting:
            byte_read = self.ser.read(1)
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
                    return data
                else:
                    self.serial_read_buffer.extend(byte_read)

def main():
    rclpy.init()
    node = CommunicationDriver()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()