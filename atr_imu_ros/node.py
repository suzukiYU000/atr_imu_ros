"""ROS 2 Jazzy adapter. Bluetooth work never blocks the executor."""
import threading
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import Imu
from diagnostic_msgs.msg import DiagnosticArray
from .configuration import load_config
from .driver import Manager
from .messages import imu_message, diagnostics

class ImuNode(Node):
    def __init__(self):
        super().__init__('atr_imu_ros')
        path = self.declare_parameter('config_file','').value
        self.config = load_config(path)
        self.publishers_by_name = {}
        self.publisher_lock = threading.Lock()
        self.diagnostic_pub = self.create_publisher(DiagnosticArray,'diagnostics',10)
        self.manager = Manager(self.config,self.publish_sample,lambda:self.get_clock().now().nanoseconds)
        self.timer = self.create_timer(1.0,self.publish_diagnostics)
        self.manager.start()
        self.get_logger().info('AMWS020 driver started; topic prefix: '+self.config['topic_prefix'])

    def publish_sample(self,device,stamp_ns,acc,gyro):
        with self.publisher_lock:
            if device.name not in self.publishers_by_name:
                topic = self.config['topic_prefix']+'/'+device.name+'/data_raw'
                self.publishers_by_name[device.name] = self.create_publisher(Imu,topic,QoSProfile(depth=50,reliability=ReliabilityPolicy.BEST_EFFORT))
                self.get_logger().info('Publishing '+topic)
            pub = self.publishers_by_name[device.name]
        pub.publish(imu_message(device,Time(nanoseconds=stamp_ns).to_msg(),acc,gyro))

    def publish_diagnostics(self):
        self.diagnostic_pub.publish(diagnostics(self.manager,self.get_clock().now().to_msg()))

    def destroy_node(self):
        self.manager.close()
        return super().destroy_node()

def main(args=None):
    # Keep context alive until workers have sent stop commands.
    from rclpy.signals import SignalHandlerOptions
    import signal
    rclpy.init(args=args,signal_handler_options=SignalHandlerOptions.NO)
    node = None
    interrupted = threading.Event()
    old_int = signal.signal(signal.SIGINT,lambda *_:interrupted.set())
    old_term = signal.signal(signal.SIGTERM,lambda *_:interrupted.set())
    try:
        node = ImuNode()
        while rclpy.ok() and not interrupted.is_set(): rclpy.spin_once(node,timeout_sec=0.2)
    finally:
        if node: node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
        signal.signal(signal.SIGINT,old_int)
        signal.signal(signal.SIGTERM,old_term)
