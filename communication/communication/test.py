import unittest
from unittest.mock import MagicMock
from serial_parser import Message

class TestMessage(unittest.TestCase):

    def test_serialize(self):
        message = Message(1, [1.0, 2.0], [1, 2], [1.0, 2.0])
        serialized = message.serialize()
        self.assertIsInstance(serialized, bytes)

    def test_deserialize(self):
        data = b'\x01\x00\x00\x80?\x00\x00\x00@\x01\x02\x00\x00\x80?\x00\x00\x00@'
        message = Message.deserialize(data)
        self.assertIsInstance(message, Message)
        self.assertEqual(message.agent_id, 1)
        self.assertEqual(message.winning_bids, [1.0, 2.0])
        self.assertEqual(message.winning_agents, [1, 2])
        self.assertEqual(message.timestamps, [1.0, 2.0])

    def test_add_escape_to_message(self):
        data = b'\x02\x03\x1B'
        escaped_data = Message.add_escape_to_message(data)
        self.assertEqual(escaped_data, b'\x02\x1B\x02\x1B\x03\x1B\x1B\x03')

    def test_remove_escape_from_message(self):
        data = b'\x02\x1B\x02\x1B\x03\x1B\x1B\x03'
        unescaped_data = Message.remove_escape_from_message(data)
        self.assertEqual(unescaped_data, b'\x02\x03\x1B')

if __name__ == '__main__':
    unittest.main()
