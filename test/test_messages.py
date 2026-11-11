import unittest
from atr_imu_ros.configuration import device
from atr_imu_ros.messages import imu_message
from sensor_msgs.msg import Imu

class MessageTests(unittest.TestCase):
    def test_imu_contract(self):
        stamp=Imu().header.stamp
        msg=imu_message(device('00:11:22:33:44:55','base'),stamp,(1.,2.,3.),(4.,5.,6.))
        self.assertEqual(msg.header.frame_id,'base_link')
        self.assertEqual(msg.orientation_covariance[0],-1.)
        self.assertEqual(msg.orientation.w,1.)
        self.assertEqual(list(msg.linear_acceleration_covariance),[0.]*9)
        self.assertEqual(msg.angular_velocity.z,6.)

if __name__=='__main__': unittest.main()
