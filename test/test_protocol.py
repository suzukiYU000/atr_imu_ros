import math
import struct
import threading
import time
import unittest
from atr_imu_ros.protocol import Parser, packet, decode_imu, SensorClock, ProtocolError, MeasurementEnded
from atr_imu_ros.configuration import device, load_config
from atr_imu_ros.driver import Manager, Worker

class ProtocolTests(unittest.TestCase):
    def test_fragmented_coalesced_corrupt_frames(self):
        p = Parser()
        data = packet(0x80,struct.pack('<I',500)+b'\x00'*18)
        p.buffer.extend(data[:1]); self.assertIsNone(p.pop())
        p.buffer.extend(data[1:8]); self.assertIsNone(p.pop())
        p.buffer.extend(data[8:]+bytes([154,188,2,0])+packet(0xbc,b'\x02'))
        self.assertEqual(p.pop()[0],0x80)
        self.assertEqual(p.pop(),(0xbc,b'\x02'))
        self.assertEqual(p.discarded_bytes,4)

    def test_si_units_and_sign(self):
        raw = [10000,-10000,0,9000,-18000,-1]
        tick,acc,gyro = decode_imu(struct.pack('<I',10)+b''.join(v.to_bytes(3,'little',signed=True) for v in raw))
        self.assertEqual(tick,10)
        self.assertAlmostEqual(acc[0],9.80665)
        self.assertAlmostEqual(acc[1],-9.80665)
        self.assertAlmostEqual(gyro[0],math.pi/2)
        self.assertAlmostEqual(gyro[1],-math.pi)
        self.assertLess(gyro[2],0)

    def test_midnight_gaps_reset(self):
        clock = SensorClock()
        self.assertEqual(clock.stamp(86399990,1000000000,10),1000000000)
        self.assertEqual(clock.stamp(0,9999999999,10),1010000000)
        self.assertEqual(clock.stamp(20,9999999999,10),1030000000)
        self.assertEqual(clock.gaps,1)
        with self.assertRaises(ProtocolError): clock.stamp(19,0,10)

    def test_config_validation(self):
        import tempfile
        from pathlib import Path
        for content in ['sample_period_ms: 0','auto_discover: yesplease','reconnect_delay: .nan','topic_prefix: /bad',
                        'sensors:\n - address: "00:11:22:33:44:55"\n   name: base\n - address: "00:11:22:33:44:56"\n   name: base']:
            with tempfile.TemporaryDirectory() as temp:
                p=Path(temp)/'config.yaml';p.write_text(content)
                with self.assertRaises(ValueError): load_config(str(p))

class FakeWorker:
    def __init__(self,d,*args): self.device=d; self.started=0
    def start(self): self.started+=1
    def snapshot(self): return {}

class ManagerTests(unittest.TestCase):
    def test_two_devices_and_discovery_dedup(self):
        a=device('00:11:22:33:44:55','base');b=device('00:11:22:33:44:66','arm')
        class Discovery:
            def devices(self): return [a,b]
        c=load_config();c['sensors']=[a]
        m=Manager(c,None,None,Discovery(),FakeWorker)
        m.scan();m.scan()
        self.assertEqual(len(m.workers),2)
        self.assertTrue(all(w.started==1 for w in m.workers.values()))

    def test_explicit_devices_survive_discovery_failure(self):
        class Broken:
            def devices(self): raise OSError('DBus unavailable')
        c=load_config();c['sensors']=[device('00:11:22:33:44:55','base')]
        m=Manager(c,None,None,Broken(),FakeWorker);m.scan()
        self.assertEqual(len(m.workers),1)
        self.assertIn('DBus unavailable',m.error)

    def test_worker_reconnect_and_shutdown(self):
        instances=[];samples=[]
        class FakeSensor:
            def __init__(self,*args):
                self.parser=Parser();self.stopped=False;self.closed=False;self.count=0
                instances.append(self)
            def connect(self):
                if len(instances)==1: raise ConnectionError('lost')
            def configure(self,p): return {'serial':'fake'}
            def start(self): pass
            def frame(self,timeout=None):
                self.count+=1
                return 0x80,struct.pack('<I',self.count*10)+b'\0'*18
            def stop(self): self.stopped=True
            def close(self): self.closed=True
        c=load_config();c['reconnect_delay']=0.01
        w=Worker(device('00:11:22:33:44:55'),c,lambda *args:(samples.append(args),w.stopping.set()),lambda:10**9,FakeSensor)
        w.loop()
        self.assertEqual(len(instances),2)
        self.assertEqual(len(samples),1)
        self.assertTrue(all(i.closed and i.stopped for i in instances))
        self.assertEqual(w.snapshot()['reconnects'],1)

if __name__=='__main__': unittest.main()
