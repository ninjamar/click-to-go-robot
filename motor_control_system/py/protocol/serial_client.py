"""Synchronous request/response client for the firmware's serial protocol."""

import time

import serial

from .protocol import RESPONSE_SIZE, build_command_packet, parse_response_packet


class SerialClient:
    """Owns one serial connection plus its outgoing sequence-id counter."""

    def __init__(self, port, baud, timeout=2):
        self.conn = serial.Serial(port, baud, timeout=timeout)
        time.sleep(2)  # Arduino resets on serial open; wait for firmware to boot
        self.conn.reset_input_buffer()
        self._next_seq_id = 1

    def __enter__(self):
        return self

    def __exit__(self, *exc_info):
        self.close()

    def close(self):
        self.conn.close()

    def _alloc_seq_id(self):
        seq_id = self._next_seq_id
        self._next_seq_id += 1
        return seq_id

    def send(self, op, arg=0.0, direction=0, seq_id=None):
        """Write one command packet; returns the seq_id used."""
        if seq_id is None:
            seq_id = self._alloc_seq_id()
        self.conn.write(build_command_packet(seq_id, op, arg, direction))
        return seq_id

    def read_response(self, timeout=2):
        """Read+parse the next response packet, waiting up to `timeout` seconds."""
        self.conn.timeout = timeout
        data = b""
        while len(data) < RESPONSE_SIZE:
            chunk = self.conn.read(RESPONSE_SIZE - len(data))
            if not chunk:
                break
            data += chunk
        return parse_response_packet(data)

    def _call(self, op, arg=0.0, direction=0, seq_id=None, timeout=2):
        """Send one command and block for its response."""
        seq_id = self.send(op, arg, direction, seq_id=seq_id)
        return self.read_response(timeout=timeout)

    def call(
        self,
        op,
        *,
        seq_id=None,
        direction=0,
        duration=None,
        distance=None,
        angle=None,
        rotations=None,
        pwm=None,
        timeout=2,
    ):
        """Send one command with human-readable args; for queued ops this is just the "queued" ack."""
        values = [
            v for v in (duration, distance, angle, rotations, pwm) if v is not None
        ]
        assert len(values) <= 1, (
            "pass at most one of duration/distance/angle/rotations/pwm"
        )
        arg = values[0] if values else 0.0
        return self._call(op, arg, direction, seq_id=seq_id, timeout=timeout)

    def drain(self, timeout=0.2):
        """Discard any buffered response bytes for up to `timeout` seconds."""
        self.conn.timeout = timeout
        while self.conn.read(RESPONSE_SIZE):
            pass
