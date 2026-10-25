"""Serial transport layer for the motor control firmware."""

import queue
import threading
import time

import serial
from protocol import RESPONSE_SIZE, build_command_packet, parse_response_packet

RECONNECT_DELAY_S = 1.0


class MotorSerial:
    """Manages the serial connection, reader thread, and packet framing.

    Separates low-level serial concerns from the ROS2 node. The node calls
    send() to write commands and drain() to collect responses that have arrived
    since the last drain. If the USB connection drops, the reader thread reopens it
    (on whatever port find_port returns) and bumps `generation`.
    """

    def __init__(self, port: str, baud: int, find_port=None, boot_delay: float = 2.0):
        self._port = port
        self._baud = baud
        self._find_port = find_port  # returns the current port path after a reconnect
        self._boot_delay = boot_delay
        self._lock = threading.RLock()
        self._seq = 0
        self._queue: queue.SimpleQueue = queue.SimpleQueue()
        self._conn = None
        # Bumped on every (re)connect; the Arduino resets with each one, so anything the
        # caller learned from the firmware (cumulative odometry, queued commands) is gone.
        self.generation = 0
        self._open()
        threading.Thread(target=self._reader, daemon=True).start()

    @property
    def connected(self) -> bool:
        return self._conn is not None

    def send(self, op, arg: float = 0.0, direction: int = 0) -> int | None:
        """Write a command packet; returns the seq_id assigned to it, or None if the
        link is down."""
        with self._lock:
            if self._conn is None:
                return None
            self._seq = (self._seq + 1) % 256
            try:
                self._conn.write(build_command_packet(self._seq, op, arg, direction))
            except (serial.SerialException, OSError):
                self._drop()
                return None
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

    def _open(self) -> None:
        port = self._port
        if self._find_port is not None:
            try:
                port = self._find_port()
            except RuntimeError:
                pass  # keep the last known path
        conn = serial.Serial(port, self._baud, timeout=5)
        time.sleep(self._boot_delay)  # Arduino resets on serial open; wait for firmware to boot
        conn.reset_input_buffer()
        with self._lock:
            self._port = port
            self._conn = conn
            self.generation += 1

    def _drop(self) -> None:
        """Close a connection that failed; the reader thread reopens it."""
        with self._lock:
            conn, self._conn = self._conn, None
        if conn is not None:
            try:
                conn.close()
            except (serial.SerialException, OSError):
                pass

    def _reader(self) -> None:
        """Background thread: accumulate bytes and enqueue complete packets, and
        reopen the connection after it drops."""
        buf = b""
        while True:
            conn = self._conn
            if conn is None:
                buf = b""
                time.sleep(RECONNECT_DELAY_S)
                try:
                    self._open()
                except (serial.SerialException, OSError):
                    pass  # still gone; try again
                continue
            try:
                chunk = conn.read(RESPONSE_SIZE - len(buf))
                if chunk:
                    buf += chunk
                if len(buf) >= RESPONSE_SIZE:
                    self._queue.put(parse_response_packet(buf[:RESPONSE_SIZE]))
                    buf = buf[RESPONSE_SIZE:]
            except (serial.SerialException, OSError):
                self._drop()
            except Exception:
                buf = b""  # garbled packet; resync on the next one
