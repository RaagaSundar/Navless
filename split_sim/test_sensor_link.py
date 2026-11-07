import socket
import unittest
import zlib
from sensor_link import HEADER, MAX_BYTES, TOPICS, decode, encode, read_exact


class SensorProtocolTests(unittest.TestCase):
    def test_all_message_types_roundtrip_binary(self):
        for index in range(len(TOPICS)):
            payload = bytes(range(256)) * 100
            packet = encode(index, payload)
            topic, size = HEADER.unpack(packet[:HEADER.size])
            self.assertEqual(len(packet) - HEADER.size, size)
            self.assertEqual(decode(topic, packet[HEADER.size:]), payload)

    def test_rejects_unlisted_topics_and_large_payloads(self):
        for index in [-1, len(TOPICS), 255]:
            with self.assertRaises(ValueError):
                encode(index, b'x')
            with self.assertRaises(ValueError):
                decode(index, zlib.compress(b'x'))
        with self.assertRaises(ValueError):
            encode(0, b'x' * (MAX_BYTES + 1))

    def test_rejects_truncation_bombs_and_trailing_stream(self):
        for body in [b'', zlib.compress(b'x')[:-1],
                     zlib.compress(b'x' * (MAX_BYTES + 1)),
                     zlib.compress(b'x') + zlib.compress(b'y')]:
            with self.assertRaises(ValueError):
                decode(0, body)

    def test_closed_connection_never_repeats_partial_message(self):
        one, two = socket.socketpair()
        with one, two:
            one.sendall(b'abc')
            one.shutdown(socket.SHUT_WR)
            self.assertEqual(read_exact(two, 3), b'abc')
            with self.assertRaises(ConnectionError):
                read_exact(two, 1)


if __name__ == '__main__':
    unittest.main()
