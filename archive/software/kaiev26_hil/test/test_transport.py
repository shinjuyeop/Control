from kaiev26_hil.transport import SerialCommandClient


class FakeSerial:
    def __init__(self, *args, **kwargs):
        self.is_open = True
        self.writes = []
        self.responses = []

    def reset_input_buffer(self):
        pass

    def write(self, data):
        self.writes.append(data)
        command = data.decode().strip()
        self.responses.append((f"OK {command}\r\n").encode())

    def flush(self):
        pass

    def readline(self):
        return self.responses.pop(0) if self.responses else b""

    def close(self):
        self.is_open = False


def test_serial_client_writes_lf_and_parses_response():
    instances = []
    def factory(*args, **kwargs):
        instance = FakeSerial(*args, **kwargs)
        instances.append(instance)
        return instance

    client = SerialCommandClient("fake", serial_factory=factory)
    client.open()
    response = client.request("set mode auto")
    assert instances[0].writes == [b"SET MODE AUTO\n"]
    assert response.kind == "ok"
    assert response.ok_command == "SET MODE AUTO"
