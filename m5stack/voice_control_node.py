import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy
import time


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
        
        # topicごとの動作関数を登録 
        self.commands = {
            0:  ("STOP",        self.action_stop), # 停止
            11: ("FORWARD",     self.action_forward), # 直進
            1:  ("SLOW",        self.action_slow), # ゆっくり直進
            3:  ("TURN RIGHT",  self.action_turn_right), # 90度右に旋回
            4:  ("TURN LEFT",   self.action_turn_left), # 90度左に旋回
            10: ("BACK",        self.action_back), # 後退
            99: ("SPIN ONCE",   self.action_spin_once), # 左に一回転
        }
        
        self.get_logger().info('Voice Control Node started')
    
    def listener_callback(self, msg): # topic受信時に呼ばれるやつ
        self.get_logger().info(f'Received trigger: {msg.data}')

        if msg.data not in self.commands: #コマンドを設定していなかった場合
            self.get_logger().warn(f"未設定のコマンド: {msg.data}")
            return

        name, func = self.commands[msg.data]

        # 動作中に別の動作を受けたら安全停止を挟む
        if self.moving and msg.data != 0:
            self.safe_stop()

        self.get_logger().info(f"Executing command: {name}")
        func()  # ← 登録された関数を実行！    
        


    # ========== 動作ユーティリティ ==========
    def publish_cmd(self, lin, ang):
        twist = Twist()
        twist.linear.x = lin
        twist.angular.z = ang
        self.publisher.publish(twist)

        # movingフラグ管理
        self.moving = not (lin == 0.0 and ang == 0.0)

    def safe_stop(self):
        self.get_logger().info("安全停止: STOP")
        self.publish_cmd(0.0, 0.0)
        time.sleep(0.5)

    # ========== 基本動作 ==========
    def action_stop(self): # 停止
        self.publish_cmd(0.0, 0.0) 

    def action_forward(self): # 直進
        self.publish_cmd(0.1, 0.0)

    def action_slow(self): # ゆっくり直進
        self.publish_cmd(0.05, 0.0)

    # def action_turn_right(self): # 右旋回
    #     self.publish_cmd(0.0, -0.4)

    # def action_turn_left(self): # 左旋回 
    #     self.publish_cmd(0.0, 0.4)

    def action_back(self): # 後退
        self.publish_cmd(-0.1, 0.0)

    # ========== ★追加可能：複雑な動作 ==========
    def action_spin_once(self):
        # 1回転して停止（例）
        self.get_logger().info("Performing 360-degree spin")

        # 1回転 ≒ 2π rad
        # angular_z = 1.0 rad/s → 1回転に約 6.3 sec
        self.publish_cmd(0.0, 1.0)
        time.sleep(6.3)

        self.action_stop()

    def action_turn_right(self): # 右旋回
        self.publish_cmd(0.0, -1.0)
        time.sleep(1.57) #45度回転

        self.action_stop()

    def action_turn_left(self): # 左旋回 
        self.publish_cmd(0.0, 1.0)
        time.sleep(1.57) #45度回転

        self.action_stop()
                                                                                                                                  

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
