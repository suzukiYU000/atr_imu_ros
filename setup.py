from glob import glob
from setuptools import setup
setup(name='atr_imu_ros',version='0.1.0',packages=['atr_imu_ros'],
      data_files=[('share/ament_index/resource_index/packages',['resource/atr_imu_ros']),
                  ('share/atr_imu_ros',['package.xml']),('share/atr_imu_ros/config',glob('config/*.yaml')),
                  ('share/atr_imu_ros/launch',glob('launch/*.launch.py'))],
      install_requires=['setuptools'],zip_safe=True,
      maintainer='suzukiYU000',maintainer_email='126460582+suzukiYU000@users.noreply.github.com',
      description='Automatic multi-AMWS020 Bluetooth IMU sensor driver',license='Proprietary',
      entry_points={'console_scripts':['imu_node = atr_imu_ros.node:main']})
