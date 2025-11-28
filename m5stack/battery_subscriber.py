import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from rclpy.qos import QoSProfile, ReliabilityPolicy

class BatterySubscriber(Node):
    def __init__(self):
        super().__init__('battery_subscriber')

        # QoSをBEST_EFFORTに設定
        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = ReliabilityPolicy.BEST_EFFORT

        self.subscription = self.create_subscription(
            Int32,
            'voice_trigger',
            self.listener_callback,
            qos_profile)
        self.subscription

    def listener_callback(self, msg):
        self.get_logger().info(f'BUTTON: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = BatterySubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
