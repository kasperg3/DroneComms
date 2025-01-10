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
    winning_bids: list
    winning_agents: list


def serialize_message(message: Message):
    bids_data = struct.pack(f"{len(message.winning_bids)}f", *message.winning_bids)
    agents_data = struct.pack(
        f"{len(message.winning_agents)}B", *message.winning_agents
    )
    return bids_data + agents_data


def deserialize_message(data):
    n_bids = len(data) // 5
    bids_data = data[: n_bids * 4]
    agents_data = data[n_bids * 4 :]
    winning_bids = list(struct.unpack(f"{n_bids}f", bids_data))
    winning_agents = list(struct.unpack(f"{n_bids}B", agents_data))
    return Message(winning_bids, winning_agents)


def send_message_to_serial(port, message: Message):
    data = serialize_message(message)
    print(f"Sending {len(data)} of data to serial port")
    if len(data) > 1490:
        raise ValueError("Data size exceeds 1490 bytes")
    with serial.Serial(port, timeout=0.01) as ser:
        ser.write(data)
        ser.flush()


def receive_message_from_serial(port):
    with serial.Serial(port, timeout=0.05) as ser:
        start_sequence = b"\x02\x02\x02"
        stop_sequence = b"\x03\x03\x03"
        buffer = bytearray()
        while True:
            ser.read_until(start_sequence)
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
            message = deserialize_message(data)
            return message


if __name__ == "__main__":
    port = "/dev/ttyACM0"  # Change this to your serial port
    lower_bound = 0
    upper_bound = 100
    n_tasks = 280

    winning_bids = generate_random_floats(n_tasks, lower_bound, upper_bound)
    winning_agents = [random.randint(0, 10) for _ in range(n_tasks)]

    message = Message(winning_bids, winning_agents)



    def send_periodically():
        while True:
            start_time = time.time()
            send_message_to_serial(port, message)
            received_message = receive_message_from_serial("/dev/ttyACM1")
            end_time = time.time()

            elapsed_time = end_time - start_time
            print(f"Elapsed time: {1/elapsed_time:.2f} Hz")


    # with serial.Serial("/dev/ttyACM1") as ser:
    #     ser.reset_input_buffer()
    #     ser.reset_output_buffer()

    send_thread = threading.Thread(target=send_periodically)
    send_thread.start()
