"""High-level configuration operations shared by CLI and GUI."""

import time

from .parameters import PARAMETERS
from .protocol import BURN_EEPROM, make_write_frame
from .transport import KeyaSerialTransport


class KeyaController:
    def __init__(self, port, *, debug=False, transport_factory=KeyaSerialTransport, sleep=time.sleep):
        self.port = port
        self.transport = transport_factory(port, debug=debug)
        self._sleep = sleep

    def connect(self):
        self.transport.enter_configuration()
        return self.transport.read_cycle(required_indices=PARAMETERS)

    def read(self):
        return self.transport.read_cycle(fresh=True, required_indices=PARAMETERS)

    def apply_ram(self, values, *, displayed=None):
        snapshot = self.read()
        for index in values:
            if index not in PARAMETERS or PARAMETERS[index].read_only:
                raise ValueError(f"{index:04d}: parameter is not writable")
            if (index, 1) not in snapshot:
                raise ValueError(f"{index:04d}: RAM value was not received")
            if displayed is not None and displayed.get((index, 1)) != snapshot[(index, 1)]:
                raise ValueError(f"{index:04d}: RAM changed since the displayed read; refresh first")
        changed = [index for index, raw in values.items() if snapshot[(index, 1)] != raw]
        try:
            for index in changed:
                self.transport.send(make_write_frame(index, values[index]))
                snapshot = self.read()
                if snapshot.get((index, 1)) != values[index]:
                    raise IOError(f"{index:04d}: expected RAM readback was not observed")
        except (OSError, TimeoutError) as exc:
            raise IOError(f"RAM apply incomplete; some values may have changed. Read again. {exc}") from exc
        return snapshot

    def burn(self, *, displayed=None):
        snapshot = self.read()
        expected = {index: raw for (index, bank), raw in snapshot.items()
                    if bank == 1 and index in PARAMETERS}
        if not expected or any((index, 2) not in snapshot for index in expected):
            raise ValueError("RAM/ROM read incomplete; EEPROM storage was not started")
        if displayed is not None:
            prior = {index: raw for (index, bank), raw in displayed.items()
                     if bank == 1 and index in PARAMETERS}
            if prior != expected:
                raise ValueError("RAM changed since the displayed read; refresh before storing")
        self.transport.send(BURN_EEPROM)
        self._sleep(5.0)  # V4.3 WriteDevtimer.Interval is 5000 ms; this is not an ACK.
        try:
            snapshot = self.read()
        except (OSError, TimeoutError) as exc:
            raise IOError("Burn was sent; EEPROM result is unknown. Read again before retrying.") from exc
        mismatch = [index for index, raw in expected.items() if snapshot.get((index, 2)) != raw]
        if mismatch:
            ids = ", ".join(f"{index:04d}" for index in mismatch)
            raise IOError(f"Burn was sent; ROM verification failed for {ids}. Read again before retrying.")
        return snapshot

    def close(self):
        self.transport.close()

    def __enter__(self):
        return self

    def __exit__(self, _type, _value, _traceback):
        try:
            self.close()
        except OSError:
            if _type is None:
                raise
