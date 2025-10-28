import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist #速度指令消息类型

class KeyboardController(Node):
    def __init__(self):
        super().__init__('keyboard_controller') #节点名称
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10) # 发布速度指令话题
        self.get_logger().info('键盘控制节点已启动，按 WASD ，Q 退出')

    def run(self):
        while rclpy.ok():
            key = input("输入指令 (W: 前进，S:后退，A:左转，D:右转，Q:退出)").lower()
            twist = Twist()
            if key == "w":
                twist.linear.x = 0.2
                self.get_logger().info(' twist.linear.x = 0.2 ') 
            elif key == 's':
                twist.linear.x = -0.2
                self.get_logger().info(' twist.linear.x =-0.2 ')
            elif key == 'a':
                twist.angular.z = 0.5 #角速度 (左转)
                self.get_logger().info(' twist.angular.z = 0.5 ')
            elif key == 'd':
                twist.angular.z = -0.5
                self.get_logger().info(' twist.angular.z = -0.5 ')
            elif key == 'q':
                break
            else:
                self.get_logger().info('无效指令')
                continue
            self.cmd_vel_pub.publish(twist) #发布速度指令
    
def main(args=None):
    rclpy.init(args=args)
    node = KeyboardController()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
