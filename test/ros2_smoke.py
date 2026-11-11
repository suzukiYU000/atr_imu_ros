"""ROS graph smoke test with synthetic samples; does not access hardware."""
import tempfile
from pathlib import Path
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
qos_profile_sensor_data = QoSProfile(depth=50,reliability=ReliabilityPolicy.BEST_EFFORT)
from sensor_msgs.msg import Imu
from atr_imu_ros.node import ImuNode
from atr_imu_ros.configuration import device

with tempfile.TemporaryDirectory() as temp:
    config=Path(temp)/'config.yaml'; config.write_text('auto_discover: false\nsensors: []\n')
    rclpy.init(args=['--ros-args','-p','config_file:='+str(config)])
    driver=ImuNode(); listener=Node('atr_smoke_listener')
    received={name:[] for name in ('base','arm')}
    subscriptions=[listener.create_subscription(Imu,'imu/'+name+'/data_raw',lambda msg,n=name:received[n].append(msg),qos_profile_sensor_data) for name in received]
    try:
        deadline=time.monotonic()+8
        while time.monotonic()<deadline and not all(received.values()):
            for i,name in enumerate(received):
                driver.publish_sample(device('00:11:22:33:44:0'+str(i),name),1234567890,(0.,0.,9.80665),(0.,0.,1.0))
            rclpy.spin_once(listener,timeout_sec=0.1)
        assert all(received.values()),received
        for name,messages in received.items():
            msg=messages[0]
            assert msg.header.frame_id==name+'_link'
            assert msg.orientation_covariance[0]==-1
            assert msg.header.stamp.sec==1 and msg.header.stamp.nanosec==234567890
            assert msg.angular_velocity.z==1.0
        print('ROS 2 graph PASS: two sensor topics, SI units, timestamps, distinct frame IDs')
    finally:
        driver.destroy_node();listener.destroy_node();rclpy.shutdown()
