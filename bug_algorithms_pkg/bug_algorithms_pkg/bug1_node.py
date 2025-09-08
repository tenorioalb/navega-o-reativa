import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Header
import math

class Bug1Node(Node):
    def __init__(self):
        super().__init__('bug1_node')

        # Parâmetros
        self.goal_x = 5.0 # Ponto de destino X (ajuste conforme seu mundo)
        self.goal_y = -3.0 # Ponto de destino Y (ajuste conforme seu mundo)
        self.dist_to_wall = 0.8 # Distância segura da parede
        self.kp_linear = 0.5 # Ganho proporcional para velocidade linear
        self.kp_angular = 0.6 # Ganho proporcional para velocidade angular

        # Estado da máquina de estados
        self.state = "GO_TO_GOAL"
        self.start_point_x = 0.0
        self.start_point_y = 0.0
        self.leave_point_x = 0.0
        self.leave_point_y = 0.0

        # Publicadores e Subscritores
        self.publisher_ = self.create_publisher(TwistStamped, 'cmd_vel', 10)
        self.state_publisher_ = self.create_publisher(String, 'bug1_state', 10)
        self.subscription_scan = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            10)
        self.subscription_odom = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10)
        
        self.timer_ = self.create_timer(0.1, self.timer_callback)

        self.get_logger().info('Bug1 node started')
    
    def odom_callback(self, msg):
        # A odometria é usada para rastrear a posição do robô
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        self.robot_theta = self.quat_to_euler(msg.pose.pose.orientation)[2]
        
    def scan_callback(self, msg):
        self.lidar_data = msg.ranges

    def quat_to_euler(self, quat):
        # Converte quaternion para ângulos de Euler (roll, pitch, yaw)
        sinr_cosp = 2 * (quat.w * quat.x + quat.y * quat.z)
        cosr_cosp = 1 - 2 * (quat.x * quat.x + quat.y * quat.y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        
        sinp = 2 * (quat.w * quat.y - quat.z * quat.x)
        pitch = math.asin(sinp)
        
        siny_cosp = 2 * (quat.w * quat.z + quat.x * quat.y)
        cosy_cosp = 1 - 2 * (quat.y * quat.y + quat.z * quat.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        return roll, pitch, yaw

    def timer_callback(self):
        # Cria a mensagem TwistStamped, que inclui a informação de tempo e o frame
        twist_stamped_msg = TwistStamped()

        # Define o cabeçalho com o tempo atual
        twist_stamped_msg.header.stamp = self.get_clock().now().to_msg()
        twist_stamped_msg.header.frame_id = 'base_link' 
        
        # Adicionando uma verificação para garantir que o robô não comece a se mover sem dados
        if not hasattr(self, 'lidar_data') or not hasattr(self, 'robot_x'):
            return

        min_dist_front = min(self.lidar_data[0:90] + self.lidar_data[270:360])
        dist_to_goal = math.sqrt((self.goal_x - self.robot_x)**2 + (self.goal_y - self.robot_y)**2)
        
        if dist_to_goal < 0.2:
            self.get_logger().info("Goal reached!")
            twist_stamped_msg.twist.linear.x = 0.0
            self.publisher_.publish(twist_stamped_msg)
            return

        if self.state == "GO_TO_GOAL":
            if min_dist_front < self.dist_to_wall:
                self.get_logger().info("Obstacle detected! Changing to BOUNDARY_FOLLOWING.")
                self.state = "BOUNDARY_FOLLOWING"
                # Salva o ponto onde o robô deixou a M-Line
                self.leave_point_x = self.robot_x
                self.leave_point_y = self.robot_y
            else:
                angle_to_goal = math.atan2(self.goal_y - self.robot_y, self.goal_x - self.robot_x)
                angle_diff = angle_to_goal - self.robot_theta

                if angle_diff > math.pi:
                    angle_diff -= 2 * math.pi
                elif angle_diff < -math.pi:
                    angle_diff += 2 * math.pi

                twist_stamped_msg.twist.linear.x = self.kp_linear * dist_to_goal
                twist_stamped_msg.twist.angular.z = self.kp_angular * angle_diff
        
        elif self.state == "BOUNDARY_FOLLOWING":
            if min_dist_front < 0.4:
                twist_stamped_msg.twist.linear.x = 0.0
                twist_stamped_msg.twist.angular.z = 0.5
            else:
                twist_stamped_msg.twist.linear.x = 0.1
                twist_stamped_msg.twist.angular.z = -0.3

            dist_from_leave_point = math.sqrt((self.robot_x - self.leave_point_x)**2 + (self.robot_y - self.leave_point_y)**2)
            if dist_from_leave_point > 0.5:
                if min_dist_front > 1.0:
                    dist_to_goal_from_current = math.sqrt((self.goal_x - self.robot_x)**2 + (self.goal_y - self.robot_y)**2)
                    dist_to_goal_from_leave = math.sqrt((self.goal_x - self.leave_point_x)**2 + (self.goal_y - self.leave_point_y)**2)
                    
                    if dist_to_goal_from_current < dist_to_goal_from_leave:
                        self.get_logger().info("Path is clear and closer to goal! Changing to GO_TO_GOAL.")
                        self.state = "GO_TO_GOAL"

        # Publica o estado atual do robô
        state_msg = String()
        state_msg.data = self.state
        self.state_publisher_.publish(state_msg)

        self.publisher_.publish(twist_stamped_msg)

def main(args=None):
    rclpy.init(args=args)
    bug1_node = Bug1Node()
    rclpy.spin(bug1_node)
    bug1_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()