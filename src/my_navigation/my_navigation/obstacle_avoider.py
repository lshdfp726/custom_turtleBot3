import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math
import signal
import sys


class ObstacleAvoider(Node):
    def __init__(self):
       super().__init__('laser_obstacle_avoider')
       #订阅激光雷达数据
       self.laser_sub = self.create_subscription(LaserScan, '/scan', self.laser_callback, 10)

       self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

       self.safe_distance = 0.5 #避障安全距离阈值
       self.cmd = Twist()
       self.front_sector = (350, 10)  #前扇区
       self.left_sector = (80, 100)  #左扇区
       self.right_sector = (260, 280) #右扇区
       self.get_logger().warn("避障模块初始化完成")
       signal.signal(signal.SIGINT, self.signal_handler)

    def laser_callback(self, msg):
        cmd = self.cmd
        self.get_logger().info("前扇区")
        front_blocked = self.is_sector_blocked(msg, self.front_sector[0], self.front_sector[1])

        if not front_blocked:
            cmd.linear.x = 0.3
            cmd.angular.z = 0.0
            self.get_logger().warn("直行")
        else:
            self.get_logger().info("左扇区")
            left_blocked = self.is_sector_blocked(msg, self.left_sector[0], self.left_sector[1])
            self.get_logger().info("右扇区")
            right_blocked = self.is_sector_blocked(msg, self.right_sector[0], self.right_sector[1])

            if not left_blocked:
                #左侧无障碍，左转
                cmd.linear.x = 0.1
                cmd.angular.x = 0.5
                self.get_logger().info("左转")
            elif not right_blocked:
                #右侧无障碍，右转
                cmd.linear.x = 0.1
                cmd.angular.z = -0.5
                self.get_logger().info("右转")
            else:
                cmd.linear.x = -0.2
                cmd.angular.z = 0.5


        self.cmd_pub.publish(cmd)

    def is_sector_blocked(self, scan, start_deg, end_deg):
        """ 检查指定角度扇区内是否有障碍物"""
        start_rad = math.radians(start_deg)
        end_rad = math.radians(end_deg)
        wrap_around = start_rad > end_rad #是否夸0度
        for i, r in enumerate(scan.ranges):
            #计算当前角度, scan.angle_increment 雷达角度分辨率
            angle = scan.angle_min + i * scan.angle_increment
            in_sector = False
            if wrap_around:
                in_sector = angle >= start_rad or angle <= end_rad
            else:
                in_sector = start_rad <= angle <= end_rad

            if in_sector and not math.isnan(r) and not math.isinf(r) and r < self.safe_distance:
                return True

        return False
    
    def signal_handler(self, signum, frame):
        """捕获Ctrl+C，手动触发清理"""
        self.get_logger().info("收到Ctrl+C，准备清理...")
        self.cleanup()
        sys.exit(0)

    def cleanup(self):
        self.get_logger().info('直行清理操作')
        self.cmd.linear.x = 0.0
        self.cmd.angular.z = 0.0
        self.cmd_pub.publish(self.cmd)
        self.destroy_node()
        self.get_logger().info('清理完成，退出程序')

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoider()
    rclpy.spin(node)

if __name__ == '__main__':
    main()



