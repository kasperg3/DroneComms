import serial
import random
import struct
def generate_random_floats(count, lower_bound, upper_bound):
    return [random.uniform(lower_bound, upper_bound) for _ in range(count)]

BUF_SIZE = 1024

class Message:
    def __init__(self, winning_bids, winning_agents, entries):
        self.winning_bids = winning_bids
        self.winning_agents = winning_agents
        self.entries = entries

    def serialize(self):
        
        bids_data = struct.pack(f'{len(self.winning_bids)}f', *self.winning_bids)
        agents_data = struct.pack(f'{len(self.winning_agents)}B', *self.winning_agents)
        entries_data = struct.pack('I', self.entries)
        return bids_data + agents_data + entries_data

def send_message_to_serial(port, message:Message):
    data = message.serialize()
    
    with serial.Serial(port, baudrate=40000000) as ser:
        ser.write(data)
        ser.flush()
        
def send_floats_to_serial(port, floats):
    serialized_floats = ','.join(f"{f:.5f}" for f in floats)
    data = f"{serialized_floats}\n".encode()
    
    with serial.Serial(port, baudrate=40000000) as ser:
        ser.write(data)
        ser.flush()

if __name__ == "__main__":
    port = '/dev/ttyACM0'  # Change this to your serial port
    lower_bound = 10.0
    upper_bound = 99.0
    n_tasks = 20

    winning_bids = generate_random_floats(n_tasks, lower_bound, upper_bound)
    winning_agents = [random.randint(0, 255) for _ in range(n_tasks)]
    entries = random.randint(1, 100)

    message = Message(winning_bids, winning_agents, entries)
    send_message_to_serial(port, message)