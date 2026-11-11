"""Shared validated settings for both ROS editions."""
from dataclasses import dataclass
from pathlib import Path
import math
import re
import yaml

MAC = re.compile(r'(?:[0-9A-Fa-f]{2}:){5}[0-9A-Fa-f]{2}$')
NAME = re.compile(r'[A-Za-z][A-Za-z0-9_]*$')

@dataclass(frozen=True)
class Device:
    address: str
    name: str
    frame_id: str
    channel: int = 0

DEFAULTS = dict(auto_discover=True,adapter='hci0',sample_period_ms=10,rfcomm_channel=0,
                reconnect_delay=3.0,discovery_interval=5.0,read_timeout=5.0,
                topic_prefix='imu',sensors=[])

def device(address,name=None,frame_id=None,channel=0):
    address = str(address).upper()
    if not MAC.fullmatch(address):
        raise ValueError('Invalid Bluetooth address: '+address)
    name = name or 'imu_'+address.replace(':','').lower()
    if not NAME.fullmatch(name):
        raise ValueError('Sensor name must be a ROS name: '+name)
    frame_id = frame_id or name+'_link'
    if not re.fullmatch(r'[A-Za-z][A-Za-z0-9_/]*',frame_id):
        raise ValueError('Invalid frame_id: '+frame_id)
    if type(channel) is not int or not 0 <= channel <= 30:
        raise ValueError('channel must be 0 (auto) or 1..30')
    return Device(address,name,frame_id,channel)

def load_config(path=''):
    raw = yaml.safe_load(Path(path).read_text()) if path else {}
    if raw is None: raw = {}
    if not isinstance(raw,dict): raise ValueError('Config must be a mapping')
    unknown = set(raw)-set(DEFAULTS)
    if unknown: raise ValueError('Unknown config fields: '+str(unknown))
    c = dict(DEFAULTS,**raw)
    if type(c['auto_discover']) is not bool: raise ValueError('auto_discover must be boolean')
    for key,lo,hi in [('sample_period_ms',1,255),('rfcomm_channel',0,30)]:
        if type(c[key]) is not int or not lo <= c[key] <= hi: raise ValueError('Invalid '+key)
    for key in ['reconnect_delay','discovery_interval','read_timeout']:
        if not isinstance(c[key],(int,float)) or not math.isfinite(c[key]) or not 0.1 <= c[key] <= 30:
            raise ValueError(key+' must be between 0.1 and 30 seconds')
    if not isinstance(c['adapter'],str) or not re.fullmatch(r'hci\d+',c['adapter']): raise ValueError('Invalid adapter')
    if not isinstance(c['topic_prefix'],str) or not re.fullmatch(r'[A-Za-z][A-Za-z0-9_/]*',c['topic_prefix']): raise ValueError('Invalid topic_prefix')
    if not isinstance(c['sensors'],list): raise ValueError('sensors must be a list')
    devices = [device(**item) for item in c['sensors']]
    for key in ['address','name','frame_id']:
        if len({getattr(d,key) for d in devices}) != len(devices): raise ValueError('Duplicate sensor '+key)
    c['sensors'] = devices
    return c
