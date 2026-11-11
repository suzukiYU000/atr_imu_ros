"""AMWS020 command protocol v1.03. No ROS dependencies."""
import math
import socket
import struct
import time
from functools import reduce
from operator import xor

SIZES = {0x80:22, 0x81:13, 0x82:9, 0x83:7, 0x84:9, 0x85:5,
         0x87:5, 0x88:1, 0x89:1, 0x8a:30, 0x8f:1, 0x90:30,
         0x93:13, 0x97:3, 0xa3:1, 0xa6:1, 0xbb:3, 0xbc:1}

class ProtocolError(RuntimeError):
    pass

class MeasurementEnded(ProtocolError):
    def __init__(self, code):
        self.code = code
        reasons = {0:'stopped', 1:'button stop', 2:'memory full', 3:'low battery',
                   100:'start rejected', 101:'external I2C error'}
        super().__init__('Measurement ended: '+reasons.get(code,str(code)))

def packet(command, payload=b'\x00'):
    data = bytes([0x9a, command]) + payload
    return data + bytes([reduce(xor, data, 0)])

class Parser:
    def __init__(self):
        self.buffer = bytearray()
        self.discarded_bytes = 0

    def pop(self):
        while len(self.buffer) >= 2:
            if self.buffer[0] != 0x9a or self.buffer[1] not in SIZES:
                del self.buffer[0]
                self.discarded_bytes += 1
                continue
            length = SIZES[self.buffer[1]] + 3
            if len(self.buffer) < length:
                return None
            data = bytes(self.buffer[:length])
            if reduce(xor, data, 0):
                del self.buffer[0]
                self.discarded_bytes += 1
                continue
            del self.buffer[:length]
            return data[1], data[2:-1]
        return None

def decode_imu(payload):
    if len(payload) != 22:
        raise ProtocolError('IMU payload must contain 22 bytes')
    tick = struct.unpack('<I',payload[:4])[0]
    if tick >= 86400000:
        raise ProtocolError('Invalid sensor tick')
    raw = [int.from_bytes(payload[i:i+3],'little',signed=True) for i in range(4,22,3)]
    return tick, tuple(v*0.000980665 for v in raw[:3]), tuple(math.radians(v*0.01) for v in raw[3:])

class SensorClock:
    """Anchor the first sensor sample to receipt ROS time; unwrap midnight."""
    def __init__(self):
        self.previous = None
        self.stamp_ns = None
        self.gaps = 0

    def stamp(self, tick, receipt_ns, period_ms):
        if self.previous is None:
            self.stamp_ns = receipt_ns
        else:
            delta = (tick-self.previous) % 86400000
            if delta == 0 or delta > 60000:
                raise ProtocolError('Sensor clock reset or invalid timestamp interval')
            self.gaps += int(delta != period_ms)
            self.stamp_ns += delta*1000000
        self.previous = tick
        return self.stamp_ns

class Sensor:
    def __init__(self, address, channel=2, timeout=5.0):
        self.address, self.channel, self.timeout = address, channel, timeout
        self.sock = None
        self.parser = Parser()
        self.started = False

    def connect(self):
        self.sock = socket.socket(socket.AF_BLUETOOTH,socket.SOCK_STREAM,socket.BTPROTO_RFCOMM)
        self.sock.settimeout(self.timeout)
        self.sock.connect((self.address,self.channel))

    def frame(self, timeout=None):
        deadline = time.monotonic() + (self.timeout if timeout is None else timeout)
        while True:
            frame = self.parser.pop()
            if frame is not None:
                return frame
            remaining = deadline-time.monotonic()
            if remaining <= 0:
                raise TimeoutError('Sensor response timeout')
            self.sock.settimeout(remaining)
            data = self.sock.recv(4096)
            if not data:
                raise ConnectionError('Bluetooth connection closed')
            self.parser.buffer.extend(data)

    def request(self, command, response, payload=b'\x00', timeout=None):
        deadline = time.monotonic() + (self.timeout if timeout is None else timeout)
        self.sock.sendall(packet(command,payload))
        while time.monotonic() < deadline:
            code,data = self.frame(max(0.001,deadline-time.monotonic()))
            if code == 0x87:
                raise ProtocolError('Sensor error '+data.hex())
            if code == 0x89 and command != 0x15:
                raise MeasurementEnded(data[0])
            if code == 0x8f and data != b'\x00':
                raise ProtocolError('Command rejected: %02x'%command)
            if code == response:
                return data
        raise TimeoutError('Command response timeout')

    def configure(self, period_ms):
        info = self.request(0x10,0x90)
        model = info[20:30].split(b'\x00')[0].decode('ascii')
        serial = info[:10].decode('ascii')
        if not model.startswith('AMWS020'):
            raise ProtocolError('Unsupported model: '+model)
        if self.request(0x3c,0xbc) != b'\x02':
            raise ProtocolError('Device already measuring; refusing to interrupt it')
        battery = self.request(0x3b,0xbb)
        original = list(self.request(0x17,0x97))
        self.request(0x16,0x8f,bytes([period_ms,1,0]))
        return dict(serial=serial,model=model,battery_voltage=int.from_bytes(battery[:2],'little')*.01,
                    battery_percent=battery[2],original_settings=str(original))

    def start(self):
        self.started = True  # Even a lost ACK must trigger a stop attempt.
        self.request(0x13,0x93,bytes([100,0,1,1,0,0,0,0,0,1,1,0,0,0]))

    def stop(self):
        if self.started:
            self.request(0x15,0x8f,timeout=1.0)
            if self.request(0x3c,0xbc,timeout=1.0) != b'\x02':
                raise ProtocolError('Stop not confirmed')
            self.started = False

    def close(self):
        if self.sock:
            self.sock.close()
