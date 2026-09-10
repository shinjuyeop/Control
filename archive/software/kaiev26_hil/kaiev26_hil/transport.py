"""Blocking request/response transport for one STM_TEST serial connection."""

from __future__ import annotations

import time
from typing import Callable, Optional

from .models import ProtocolResponse
from .protocol import parse_response, validate_command


class SerialCommandClient:
    """Own a pyserial-like object and execute one command at a time."""

    def __init__(
        self,
        port: str,
        baud_rate: int = 115200,
        timeout_sec: float = 0.5,
        *,
        serial_factory: Optional[Callable[..., object]] = None,
    ) -> None:
        self.port = port
        self.baud_rate = int(baud_rate)
        self.timeout_sec = float(timeout_sec)
        if serial_factory is None:
            import serial
            serial_factory = serial.Serial
        self._serial_factory = serial_factory
        self._serial = None

    @property
    def is_open(self) -> bool:
        return self._serial is not None and bool(getattr(self._serial, "is_open", True))

    def open(self) -> None:
        self.close()
        self._serial = self._serial_factory(
            self.port,
            baudrate=self.baud_rate,
            bytesize=8,
            parity="N",
            stopbits=1,
            timeout=min(0.1, self.timeout_sec),
            write_timeout=self.timeout_sec,
        )
        reset = getattr(self._serial, "reset_input_buffer", None)
        if callable(reset):
            reset()

    def close(self) -> None:
        if self._serial is not None:
            try:
                self._serial.close()
            finally:
                self._serial = None

    def request(self, command: str) -> ProtocolResponse:
        if not self.is_open:
            raise ConnectionError("serial port is not open")
        normalized = validate_command(command)
        self._serial.write((normalized + "\n").encode("ascii"))
        flush = getattr(self._serial, "flush", None)
        if callable(flush):
            flush()
        deadline = time.monotonic() + self.timeout_sec
        while time.monotonic() < deadline:
            raw = self._serial.readline()
            if not raw:
                continue
            try:
                line = raw.decode("ascii", errors="strict").strip()
            except UnicodeDecodeError as exc:
                raise IOError("non-ASCII STM_TEST response") from exc
            if line:
                return parse_response(line)
        raise TimeoutError(f"STM_TEST command timed out: {normalized}")

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, exc_type, exc, traceback):
        self.close()
