"""Independent sensor workers; per-device failures do not stop other IMUs."""
import fcntl
import os
from pathlib import Path
import threading
import time
from .protocol import Sensor, SensorClock, decode_imu, MeasurementEnded, ProtocolError

class Worker(threading.Thread):
    def __init__(self, device, config, on_sample, now_ns, sensor_factory=Sensor):
        super().__init__(name=device.name,daemon=True)
        self.device,self.config = device,config
        self.on_sample,self.now_ns,self.sensor_factory = on_sample,now_ns,sensor_factory
        self.stopping = threading.Event()
        self.guard = threading.Lock()
        self.state = dict(state='waiting',message='Waiting to connect',samples=0,reconnects=0,
                          last_sample=0.0,discarded_bytes=0,timestamp_gaps=0,stop_confirmed=False)

    def update(self, **values):
        with self.guard: self.state.update(values)

    def snapshot(self):
        with self.guard: return dict(self.state)

    def run(self):
        # Lock only instances of this driver. Other applications must be stopped separately.
        lock_dir = Path(os.environ.get('ATR_IMU_LOCK_DIR','/tmp/atr_imu_ros_'+str(os.getuid())))
        lock_dir.mkdir(mode=0o700,parents=True,exist_ok=True)
        with (lock_dir/(self.device.address.replace(':','')+'.lock')).open('w') as lock:
            try: fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
            except BlockingIOError:
                self.update(state='error',message='Device is owned by another atr_imu_ros process')
                return
            self.loop()

    def loop(self):
        while not self.stopping.is_set():
            sensor = self.sensor_factory(self.device.address,self.config['rfcomm_channel'],self.config['read_timeout'])
            clock = SensorClock()
            delay = self.config['reconnect_delay']
            self.update(state='connecting',message='Connecting',stop_confirmed=False,stop_error='')
            try:
                sensor.connect()
                if self.stopping.is_set(): break
                self.update(**sensor.configure(self.config['sample_period_ms']))
                if self.stopping.is_set(): break
                sensor.start()
                self.update(state='streaming',message='Receiving IMU data')
                last_rx = time.monotonic()
                while not self.stopping.is_set():
                    try:
                        code,data = sensor.frame(timeout=min(0.25,self.config['read_timeout']))
                    except TimeoutError:
                        if time.monotonic()-last_rx >= self.config['read_timeout']:
                            raise TimeoutError('IMU data timeout')
                        continue
                    if code == 0x89: raise MeasurementEnded(data[0])
                    if code == 0x87: raise ProtocolError('Sensor error: '+data.hex())
                    if code != 0x80: continue
                    last_rx = time.monotonic()
                    tick,acc,gyro = decode_imu(data)
                    stamp = clock.stamp(tick,self.now_ns(),self.config['sample_period_ms'])
                    self.on_sample(self.device,stamp,acc,gyro)
                    state = self.snapshot()
                    self.update(samples=state['samples']+1,last_sample=time.monotonic(),
                                timestamp_gaps=clock.gaps,discarded_bytes=sensor.parser.discarded_bytes)
            except Exception as exc:
                self.update(state='error',message=str(exc))
                if isinstance(exc,MeasurementEnded) and exc.code == 3:
                    delay = max(delay,30.0)
            finally:
                try:
                    was_started = getattr(sensor,'started',False)
                    sensor.stop()
                    if was_started: self.update(stop_confirmed=True)
                except Exception as exc: self.update(stop_error=str(exc))
                sensor.close()
            if self.stopping.wait(delay): break
            self.update(reconnects=self.snapshot()['reconnects']+1)
        self.update(state='stopped',message='Driver stopped')

class Manager:
    def __init__(self, config, on_sample, now_ns, discovery=None, worker_factory=Worker):
        self.config,self.on_sample,self.now_ns = config,on_sample,now_ns
        self.discovery,self.worker_factory = discovery,worker_factory
        self.workers = {}
        self.error = ''
        self.stopping = threading.Event()
        self.guard = threading.Lock()
        self.thread = threading.Thread(target=self.run,name='imu_discovery',daemon=True)

    def start(self): self.thread.start()

    def scan(self):
        devices = list(self.config['sensors'])
        try:
            if self.config['auto_discover']:
                if self.discovery is None:
                    from .discovery import Discovery
                    self.discovery = Discovery(self.config['adapter'])
                known = {d.address for d in devices}
                devices += [d for d in self.discovery.devices() if d.address not in known]
            self.error = ''
        except Exception as exc:
            self.error = 'Discovery: '+str(exc)
        with self.guard:
            names = {w.device.name for w in self.workers.values()}
            for d in devices:
                if d.address in self.workers: continue
                if d.name in names:
                    self.error = 'Duplicate discovered name: '+d.name
                    continue
                w = self.worker_factory(d,self.config,self.on_sample,self.now_ns)
                self.workers[d.address] = w
                names.add(d.name)
                w.start()

    def run(self):
        try:
            while not self.stopping.is_set():
                self.scan()
                self.stopping.wait(self.config['discovery_interval'])
        finally:
            if self.discovery: self.discovery.close()

    def snapshot(self):
        with self.guard: return [(w.device,w.snapshot()) for w in self.workers.values()]

    def close(self):
        self.stopping.set()
        self.thread.join(5)
        with self.guard: workers = list(self.workers.values())
        for w in workers: w.stopping.set()
        deadline = time.monotonic()+self.config['read_timeout']*5+5
        for w in workers: w.join(max(0,deadline-time.monotonic()))
