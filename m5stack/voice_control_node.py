import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy



class VoiceControlNode(Node):
    def __init__(self):
        super().__init__('voice_control_node')

        self.moving =False # True:turtlebot動作中 False:停止中
        
        # QoSをBEST_EFFORTに設定
        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = ReliabilityPolicy.BEST_EFFORT
        
        # voice_trigger を sub
        self.subscription = self.create_subscription(
            Int32,
            'voice_trigger',
            self.listener_callback,
            qos_profile)
        
        # cmd_vel を pub
        self.publisher = self.create_publisher(
            Twist,
            'cmd_vel',
            10)
        
        self.get_logger().info('Voice Control Node started')
    
    def listener_callback(self, msg): # topic受信時に呼ばれるやつ
        self.get_logger().info(f'Received trigger: {msg.data}')
        
        twist = Twist()
        
        if msg.data == 0: # 停止
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.get_logger().info('Command: STOP')
            
        elif msg.data == 11: # 直進
            twist.linear.x = 0.1  # 0.2 m/s
            twist.angular.z = 0.0
            self.get_logger().info('Command: FORWARD')
            
        elif msg.data == 3: # 右旋回
           
            twist.linear.x = 0.0  
            twist.angular.z = -0.2 # マイナスは右回り
            self.get_logger().info('Command: TURN RIGHT')

        elif msg.data == 4: # 左旋回
           
            twist.linear.x = 0.1  
            twist.angular.z = 0.2 # プラスは左回り
            self.get_logger().info('Command: TURN LEFT')

        else:
            self.get_logger().warn(f'未設定の数値: {msg.data}')
            return
        
        # cmd_velにパブリッシュ
        self.publisher.publish(twist)



def main(args=None):
    rclpy.init(args=args)
    node = VoiceControlNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
