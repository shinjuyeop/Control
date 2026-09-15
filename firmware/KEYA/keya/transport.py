"""pyserial transport with one persistent receive parser."""

import time

import serial

from .protocol import DISCONNECT, ENTER_CONFIGURATION, RecordParser


class KeyaSerialTransport:
    def __init__(self, port, *, timeout=0.1, debug=False, serial_factory=serial.Serial):
        self.debug = debug
        self.entered = False
        self.parser = RecordParser()
        self._cycle_start = None
        self.received_bytes = 0
        self.decoded_records = 0
        self.recent_bytes = bytearray()
        self.serial = serial_factory(
            port=port, baudrate=115200, bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE,
            timeout=timeout, write_timeout=1.0, xonxoff=False,
            rtscts=False, dsrdtr=False, exclusive=True,
        )

    def _write(self, payload):
        if self.debug:
            print("TX:", payload.hex(" ").upper())
        if self.serial.write(payload) != len(payload):
            raise IOError("incomplete serial write; controller state is unknown")
        self.serial.flush()

    def enter_configuration(self):
        self.discard_input()
        self.serial.reset_output_buffer()
        self.entered = True
        self._write(ENTER_CONFIGURATION)

    def send(self, payload):
        if not self.entered:
            raise IOError("configuration session is not active")
        self._write(payload)

    def discard_input(self):
        self.serial.reset_input_buffer()
        self.parser.reset()
        self._cycle_start = None
        self.received_bytes = 0
        self.decoded_records = 0
        self.recent_bytes.clear()

    def _timeout_error(self, timeout, missing=None):
        if self.received_bytes == 0:
            detail = (
                "수신 바이트가 없습니다. TX/RX 교차 연결, 공통 GND, RS-232 레벨 변환, "
                "컨트롤러 정상 기동 여부를 확인하세요."
            )
        elif self.decoded_records == 0:
            tail = self.recent_bytes.hex(" ").upper()
            detail = (
                f"{self.received_bytes}바이트를 받았지만 V4.3 설정 레코드로 정렬되지 않았습니다. "
                f"마지막 수신: {tail}. 프로토콜 종류와 통신 설정을 확인하세요."
            )
        else:
            detail = (
                f"{self.received_bytes}바이트에서 설정 레코드 {self.decoded_records}개를 해석했지만 "
                "완전한 Param/RAM/ROM 주기가 끝나지 않았습니다."
            )
            if missing:
                bank_names = ("Param", "RAM", "ROM")
                shown = ", ".join(f"{bank_names[bank]} {index:04d}" for index, bank in missing[:12])
                suffix = f" 외 {len(missing) - 12}개" if len(missing) > 12 else ""
                detail += f" 아직 받지 못한 항목: {shown}{suffix}."
        return TimeoutError(f"{timeout:.1f}초 내 설정 응답을 완료하지 못했습니다. {detail}")

    def _next_record(self, deadline):
        while time.monotonic() < deadline:
            record = self.parser.pop()
            if record is not None:
                self.decoded_records += 1
                return record
            chunk = self.serial.read(max(1, self.serial.in_waiting))
            if chunk:
                self.received_bytes += len(chunk)
                self.recent_bytes.extend(chunk)
                del self.recent_bytes[:-32]
                if self.debug:
                    print("RX:", chunk.hex(" ").upper())
                self.parser.feed(chunk)
        raise TimeoutError

    def read_cycle(self, timeout=10.0, *, fresh=False, required_indices=None):
        if fresh:
            self.discard_input()
        deadline = time.monotonic() + timeout
        required = ({(index, bank) for index in required_indices for bank in range(3)}
                    if required_indices is not None else None)
        if required is not None:
            # The controller may send one finite dump, and the serial reader can
            # attach after its first record. Collect the fields the Setup consumer
            # actually needs instead of requiring a second cycle marker.
            records = {}
        elif self._cycle_start is None:
            records = None
        else:
            records = {(0, 0): self._cycle_start}
            self._cycle_start = None
        while True:
            try:
                index, raw, bank = self._next_record(deadline)
            except TimeoutError:
                missing = sorted(required - records.keys()) if required is not None else None
                raise self._timeout_error(timeout, missing) from None
            if required is not None:
                records[(index, bank)] = raw
                if required.issubset(records):
                    return records
                continue
            if index == 0 and bank == 0:
                if records is not None:
                    self._cycle_start = raw
                    return records
                records = {}
            if records is not None:
                records[(index, bank)] = raw

    def close(self):
        try:
            if self.entered and self.serial.is_open:
                self.entered = False
                self._write(DISCONNECT)
        finally:
            if self.serial.is_open:
                self.serial.close()

    def __enter__(self):
        return self

    def __exit__(self, _type, _value, _traceback):
        self.close()
