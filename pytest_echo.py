import serial
import random
import struct
from dataclasses import dataclass
import threading
def generate_random_floats(count, lower_bound, upper_bound):
    return [random.uniform(lower_bound, upper_bound) for _ in range(count)]

BUF_SIZE = 1490
@dataclass
class Message:
    winning_bids: list
    winning_agents: list

def serialize_message(message: Message):
    bids_data = struct.pack(f'{len(message.winning_bids)}f', *message.winning_bids)
    agents_data = struct.pack(f'{len(message.winning_agents)}B', *message.winning_agents)
    return  bids_data + agents_data

def deserialize_message(data):
    n_bids = len(data) // 5
    bids_data = data[:n_bids*4]
    agents_data = data[n_bids*4:]
    winning_bids = list(struct.unpack(f'{n_bids}f', bids_data))
    winning_agents = list(struct.unpack(f'{n_bids}B', agents_data))
    return Message(winning_bids, winning_agents)

def send_message_to_serial(port, message:Message):
    data = serialize_message(message)
    print(f"Sending {len(data)} of data to serial port")
    if len(data) > 1490:
        raise ValueError("Data size exceeds 1490 bytes")
    with serial.Serial(port, baudrate=40000000) as ser:
        ser.write(data)
        ser.flush()
        

def receive_message_from_serial(port):
    with serial.Serial(port, baudrate=40000000, timeout=1) as ser:
        data = ser.read(BUF_SIZE)
        if len(data) == 0:
            raise ValueError("No data received from serial port")
        print(f"Received {len(data)} of data to serial port")
        message = deserialize_message(data)
        
        return message



if __name__ == "__main__":
    port = '/dev/ttyACM0'  # Change this to your serial port
    lower_bound = 0
    upper_bound = 10
    n_tasks = 200

    winning_bids = generate_random_floats(n_tasks, lower_bound, upper_bound)
    winning_agents = [random.randint(0, 255) for _ in range(n_tasks)]

    message = Message(winning_bids, winning_agents)
    
    # def receive_thread():
    #     received_message = receive_message_from_serial("/dev/ttyACM1")
    #     print(f"Received message: {received_message}")

    # thread = threading.Thread(target=receive_thread)
    # thread.start()
    
    send_message_to_serial(port, message)