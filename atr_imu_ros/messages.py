"""ROS1/ROS2 message fields shared by both bindings."""
import time
from sensor_msgs.msg import Imu
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

def imu_message(device, stamp, acceleration, angular_velocity):
    msg = Imu()
    msg.header.stamp = stamp
    msg.header.frame_id = device.frame_id
    msg.orientation.w = 1.0
    msg.orientation_covariance[0] = -1.0
    msg.linear_acceleration.x,msg.linear_acceleration.y,msg.linear_acceleration.z = acceleration
    msg.angular_velocity.x,msg.angular_velocity.y,msg.angular_velocity.z = angular_velocity
    return msg

def diagnostics(manager,stamp):
    msg = DiagnosticArray()
    msg.header.stamp = stamp
    entries = manager.snapshot()
    root = DiagnosticStatus()
    root.name = 'atr_imu_ros/discovery'
    root.hardware_id = 'bluetooth'
    root.level = DiagnosticStatus.WARN if manager.error or not entries else DiagnosticStatus.OK
    root.message = manager.error or ('No paired AMWS020 devices found' if not entries else 'Sensors: '+str(len(entries)))
    msg.status.append(root)
    for device,state in entries:
        item = DiagnosticStatus()
        item.name = 'atr_imu_ros/'+device.name
        item.hardware_id = device.address
        age = time.monotonic()-state.get('last_sample',0)
        item.level = DiagnosticStatus.OK if state['state']=='streaming' and age<2 else DiagnosticStatus.WARN
        if state['state']=='error': item.level = DiagnosticStatus.ERROR
        item.message = state['message']
        for k,v in state.items():
            entry = KeyValue()
            entry.key,entry.value = k,str(v)
            item.values.append(entry)
        msg.status.append(item)
    return msg
