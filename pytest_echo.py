import serial
import random
import struct
from dataclasses import dataclass
import threading
import time


def generate_random_floats(count, lower_bound, upper_bound):
    return [random.uniform(lower_bound, upper_bound) for _ in range(count)]


BUF_SIZE = 1490


@dataclass
class Message:
    agent_id: int
    winning_bids: list
    winning_agents: list


def serialize_message(message: Message):
    agent_id_data = struct.pack("B", message.agent_id)
    bids_data = struct.pack(f"{len(message.winning_bids)}f", *message.winning_bids)
    agents_data = struct.pack(f"{len(message.winning_agents)}B", *message.winning_agents)
    return agent_id_data + bids_data + agents_data


def deserialize_message(data):
    agent_id = struct.unpack("B", data[:1])[0]
    data = data[1:]
    n_bids = len(data) // 5
    bids_data = data[: n_bids * 4]
    agents_data = data[n_bids * 4 :]
    winning_bids = list(struct.unpack(f"{n_bids}f", bids_data))
    winning_agents = list(struct.unpack(f"{n_bids}B", agents_data))
    return Message(agent_id, winning_bids, winning_agents)


def send_message_to_serial(port, message: Message):
    data = serialize_message(message)
    if len(data) > 1490:
        raise ValueError("Data size exceeds 1490 bytes")
    with serial.Serial(port) as ser:
        ser.write(data)
        ser.flush()


def receive_message_from_serial(port):
    with serial.Serial(port, timeout=0.05) as ser:
        start_sequence = b"\x02\x02\x02"
        stop_sequence = b"\x03\x03\x03"
        buffer = bytearray()
        while True:
            buffer.extend(ser.read(ser.in_waiting))
            if start_sequence in buffer and stop_sequence in buffer:
                start_index = buffer.find(start_sequence)
                stop_index = buffer.rfind(stop_sequence)

                if start_index < stop_index and (stop_index - start_index) < BUF_SIZE:
                    data = buffer[start_index + len(start_sequence) : stop_index]
                    break
        
        if len(data) == 0:
            print("No data received")
        else:
            try:
                message = deserialize_message(data)
            except struct.error as e:
                print(f"Failed to deserialize message: {e}")
                return None
            message = deserialize_message(data)
            return message

def sender_thread():
    sender_port = "/dev/ttyACM1"  # Change this to your serial port
    lower_bound = 0
    upper_bound = 100
    n_tasks = 200

    agent_id = random.randint(0, 255)
    winning_bids = generate_random_floats(n_tasks, lower_bound, upper_bound)
    winning_agents = [random.randint(0, 10) for _ in range(n_tasks)]
    message = Message(agent_id, winning_bids, winning_agents)

    with serial.Serial(sender_port) as ser:
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        
    data = serialize_message(message)
    def send_periodically():
        while True:
            time.sleep(1)
            print(f"Sending {len(data)} of data to serial port")
            start_time = time.time()
            send_message_to_serial(sender_port, message)
            end_time = time.time()
            elapsed_time = end_time - start_time
            print(f"Elapsed time: {1/elapsed_time:.2f} Hz")
            

    send_thread = threading.Thread(target=send_periodically)
    send_thread.start()

    
def reciever_thread():
    reciever_port = "/dev/ttyACM0"  # Change this to your serial port
    with serial.Serial(reciever_port) as ser:
        ser.reset_input_buffer()
        ser.reset_output_buffer()

    def recieve():
        while True:
            start_time = time.time()
            received_message = receive_message_from_serial(reciever_port)
            if received_message is None:
                continue
            end_time = time.time()
            if received_message:
                print(f"Received message from agent id: {received_message.agent_id}")

            elapsed_time = end_time - start_time
            print(f"Elapsed time: {1/elapsed_time:.2f} Hz")
            

    reciever_thread = threading.Thread(target=recieve)
    reciever_thread.start()


if __name__ == "__main__":
    # reciever_thread()
    sender_thread()
