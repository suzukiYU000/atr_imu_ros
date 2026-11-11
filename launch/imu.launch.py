from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    default = os.path.join(get_package_share_directory('atr_imu_ros'),'config','sensors.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('config_file',default_value=default),
        Node(package='atr_imu_ros',executable='imu_node',name='atr_imu_ros',output='screen',
             parameters=[{'config_file':LaunchConfiguration('config_file')}])])
