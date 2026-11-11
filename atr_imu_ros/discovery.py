"""List already-paired AMWS020 / TSND151 devices via BlueZ; never alter pairing."""
import re
import dbus
from .configuration import device

class Discovery:
    def __init__(self, adapter):
        self.bus = dbus.SystemBus(private=True)
        self.path = '/org/bluez/'+adapter
        self.manager = dbus.Interface(self.bus.get_object('org.bluez','/'),'org.freedesktop.DBus.ObjectManager')

    def devices(self):
        result = []
        for path, interfaces in self.manager.GetManagedObjects(timeout=3).items():
            p = interfaces.get('org.bluez.Device1',{})
            if not str(path).startswith(self.path+'/') or not p.get('Paired',False): continue
            match = re.fullmatch(r'(AMWS020|TSND151)[_-]([A-Za-z0-9]+)',str(p.get('Name','')))
            if match:
                result.append(device(str(p['Address']),'imu_'+match[2].lower(),channel={'AMWS020':2,'TSND151':1}[match[1]]))
        return result

    def close(self):
        self.bus.close()
