"""Pure byte-level KEYA V4.3 Setup protocol."""

from collections import deque

ENTER_CONFIGURATION = bytes.fromhex("FA 00 00 00 00 00 00 00")
DISCONNECT = bytes(8)
BURN_EEPROM = bytes.fromhex("FA 00 00 00 08 00 00 00")
BANK_NAMES = {0: "Param", 1: "RAM", 2: "ROM"}


def make_write_frame(parameter, raw):
    if not 0 <= parameter <= 0xFF or not 0 <= raw <= 0xFFFF:
        raise ValueError("write frame fields exceed protocol width")
    return bytes((0xF0, parameter, raw >> 8, raw & 0xFF, 0x05, 0, 0, 0))


class RecordParser:
    """Align and decode a continuous stream of four-byte configuration records."""

    def __init__(self):
        self.buffer = bytearray()
        self.records = deque()
        self.synced = False

    def reset(self):
        self.buffer.clear()
        self.records.clear()
        self.synced = False

    def feed(self, data):
        self.buffer.extend(data)
        while True:
            if not self.synced:
                if len(self.buffer) < 8:
                    return
                # Same alignment condition used by V4.3 probe_queue_get_right_data.
                if (self.buffer[3] not in BANK_NAMES or
                        self.buffer[3] != self.buffer[7] or
                        self.buffer[0] + 1 != self.buffer[4]):
                    del self.buffer[0]
                    continue
                self.synced = True
            if len(self.buffer) < 4:
                return
            if self.buffer[3] not in BANK_NAMES:
                self.synced = False
                del self.buffer[0]
                continue
            index, high, low, bank = self.buffer[:4]
            del self.buffer[:4]
            self.records.append((index, (high << 8) | low, bank))

    def pop(self):
        return self.records.popleft() if self.records else None


def read_ky(data):
    """Decode V4.3's AB 54 count + (index, high, low) backup format."""
    if len(data) < 3 or data[:2] != b"\xAB\x54":
        raise ValueError("not a KEYA .ky file (expected AB 54 header)")
    count = data[2]
    if count == 0 or len(data) != 3 + count * 3:
        raise ValueError("incomplete .ky file or incorrect record count")
    values = {}
    for offset in range(3, len(data), 3):
        index, high, low = data[offset:offset + 3]
        if index in values:
            raise ValueError(f"duplicate .ky parameter: {index:04d}")
        values[index] = (high << 8) | low
    return values


def make_ky(records, bank):
    values = sorted((index, raw) for (index, source), raw in records.items() if source == bank)
    if not values:
        raise ValueError("no configuration values to export")
    if len(values) > 0xFF:
        raise ValueError("too many configuration values for .ky format")
    return bytes((0xAB, 0x54, len(values))) + b"".join(
        bytes((index, raw >> 8, raw & 0xFF)) for index, raw in values
    )
