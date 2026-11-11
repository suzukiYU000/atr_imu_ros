"""ROS 1 Noetic adapter for the shared AMWS020 driver."""
import threading
import rospy
from sensor_msgs.msg import Imu
from diagnostic_msgs.msg import DiagnosticArray
from .configuration import load_config
from .driver import Manager
from .messages import imu_message, diagnostics

class ImuNode:
    def __init__(self):
        self.config = load_config(rospy.get_param('~config_file',''))
        self.publishers_by_name = {}
        self.publisher_lock = threading.Lock()
        self.diagnostic_pub = rospy.Publisher('diagnostics',DiagnosticArray,queue_size=10)
        self.manager = Manager(self.config,self.publish_sample,lambda:rospy.Time.now().to_nsec())
        self.timer = rospy.Timer(rospy.Duration(1),self.publish_diagnostics)
        rospy.on_shutdown(self.close)
        self.manager.start()
        rospy.loginfo('AMWS020 driver started; topic prefix: %s',self.config['topic_prefix'])

    def publish_sample(self,device,stamp_ns,acc,gyro):
        with self.publisher_lock:
            if device.name not in self.publishers_by_name:
                topic = self.config['topic_prefix']+'/'+device.name+'/data_raw'
                self.publishers_by_name[device.name] = rospy.Publisher(topic,Imu,queue_size=5)
                rospy.loginfo('Publishing %s',topic)
            pub = self.publishers_by_name[device.name]
        stamp=rospy.Time(stamp_ns//1000000000,stamp_ns%1000000000)
        pub.publish(imu_message(device,stamp,acc,gyro))

    def publish_diagnostics(self,event):
        self.diagnostic_pub.publish(diagnostics(self.manager,rospy.Time.now()))

    def close(self):
        self.timer.shutdown()
        self.manager.close()

def main():
    rospy.init_node('atr_imu_ros')
    node=ImuNode()
    rospy.spin()
