"""Serial transport layer for the motor control firmware."""

import queue
import threading
import time

import serial
from protocol import RESPONSE_SIZE, build_command_packet, parse_response_packet


class MotorSerial:
    """Manages the serial connection, reader thread, and packet framing.

    Separates low-level serial concerns from the ROS2 node. The node calls
    send() to write commands and drain() to collect responses that have arrived
    since the last drain.
    """

    def __init__(self, port: str, baud: int) -> None:
        self._conn = serial.Serial(port, baud, timeout=5)
        time.sleep(2)  # Arduino resets on serial open; wait for firmware to boot
        self._conn.reset_input_buffer()
        self._lock = threading.Lock()
        self._seq = 0
        self._queue: queue.SimpleQueue = queue.SimpleQueue()
        threading.Thread(target=self._reader, daemon=True).start()

    def send(self, op, arg: float = 0.0, direction: int = 0) -> int:
        """Write a command packet; returns the seq_id assigned to it."""
        self._seq = (self._seq + 1) % 256
        with self._lock:
            self._conn.write(build_command_packet(self._seq, op, arg, direction))
        return self._seq

    def drain(self) -> list:
        """Return all response packets received since the last call to drain()."""
        packets = []
        while not self._queue.empty():
            try:
                packets.append(self._queue.get_nowait())
            except queue.Empty:
                break
        return packets

    def _reader(self) -> None:
        """Background thread: accumulate bytes and enqueue complete packets."""
        buf = b""
        while True:
            try:
                chunk = self._conn.read(RESPONSE_SIZE - len(buf))
                if chunk:
                    buf += chunk
                if len(buf) >= RESPONSE_SIZE:
                    self._queue.put(parse_response_packet(buf[:RESPONSE_SIZE]))
                    buf = buf[RESPONSE_SIZE:]
            except Exception:
                buf = b""
