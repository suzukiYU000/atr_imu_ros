"""ROS 1 graph smoke test using synthetic samples; requires roscore."""
import tempfile
from pathlib import Path
import time
import rospy
from sensor_msgs.msg import Imu
from atr_imu_ros.node import ImuNode
from atr_imu_ros.configuration import device

with tempfile.TemporaryDirectory() as temp:
    config=Path(temp)/'config.yaml';config.write_text('auto_discover: false\nsensors: []\n')
    rospy.init_node('atr_smoke')
    rospy.set_param('~config_file',str(config))
    driver=ImuNode()
    received={name:[] for name in ('base','arm')}
    subscribers=[rospy.Subscriber('imu/'+name+'/data_raw',Imu,lambda msg,n=name:received[n].append(msg),queue_size=5) for name in received]
    try:
        deadline=time.monotonic()+8
        while time.monotonic()<deadline and not all(received.values()):
            for i,name in enumerate(received):
                driver.publish_sample(device('00:11:22:33:44:0'+str(i),name),1234567890,(0.,0.,9.80665),(0.,0.,1.))
            time.sleep(.1)
        assert all(received.values()),received
        for name,messages in received.items():
            msg=messages[0]
            assert msg.header.frame_id==name+'_link'
            assert msg.orientation_covariance[0]==-1
            assert msg.header.stamp.secs==1 and msg.header.stamp.nsecs==234567890
            assert msg.angular_velocity.z==1.
        print('ROS 1 graph PASS: two sensor topics, SI units, timestamps, distinct frame IDs')
    finally:
        rospy.signal_shutdown('test complete')
