import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import math
import numpy as np

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

class TangentBugNode(Node):
    def __init__(self):
        super().__init__('tangent_bug_node')

        # Perfis de QoS robustos
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Parâmetros do algoritmo
        self.goal_x = 2.0
        self.goal_y = 1.0
        self.dist_to_wall = 0.7  # Distância de segurança para contorno
        self.kp_linear = 0.4
        self.kp_angular = 0.8
        self.max_linear_vel = 0.25
        self.max_angular_vel = 0.7

        # Variáveis da máquina de estados
        self.state = "GO_TO_GOAL"
        self.hit_point = None
        self.d_min = float('inf')

        # Variáveis de sensores e posição
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0
        self.lidar_data = []
        self.odom_received = False
        self.scan_received = False

        # Publicadores
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', reliable_qos)
        self.state_publisher_ = self.create_publisher(String, 'bug1_state', reliable_qos)
        
        # Subscritores
        self.subscription_scan = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, sensor_qos)
        self.subscription_odom = self.create_subscription(
            Odometry, 'odom', self.odom_callback, sensor_qos)
        
        self.timer_ = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info('Nó Tangent Bug iniciado com sucesso.')

    def odom_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        self.robot_theta = self.quat_to_euler(msg.pose.pose.orientation)[2]
        if not self.odom_received:
            self.get_logger().info("Dados de odometria recebidos!")
            self.odom_received = True

    def scan_callback(self, msg):
        ranges = np.array(msg.ranges)
        ranges[np.isinf(ranges)] = msg.range_max
        ranges[np.isnan(ranges)] = msg.range_max
        self.lidar_data = ranges.tolist()
        if not self.scan_received:
            self.get_logger().info(f"Dados de LaserScan recebidos! ({len(self.lidar_data)} leituras)")
            self.scan_received = True

    def quat_to_euler(self, quat):
        sinr_cosp = 2 * (quat.w * quat.x + quat.y * quat.z)
        cosr_cosp = 1 - 2 * (quat.x * quat.x + quat.y * quat.y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        sinp = 2 * (quat.w * quat.y - quat.z * quat.x)
        sinp = max(min(sinp, 1.0), -1.0)
        pitch = math.asin(sinp)
        siny_cosp = 2 * (quat.w * quat.z + quat.x * quat.y)
        cosy_cosp = 1 - 2 * (quat.y * quat.y + quat.z * quat.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return roll, pitch, yaw

    def timer_callback(self):
        twist_msg = Twist()
        if not self.odom_received or not self.scan_received or not self.lidar_data:
            self.get_logger().warn("Aguardando dados dos sensores...", throttle_duration_sec=5)
            self.publisher_.publish(twist_msg)
            return

        dist_to_goal = math.sqrt((self.goal_x - self.robot_x)**2 + (self.goal_y - self.robot_y)**2)
        if dist_to_goal < 0.2:
            self.get_logger().info("🎯 Objetivo alcançado!")
            self.publisher_.publish(twist_msg)
            return

        # ----- LÓGICA DO TANGENT BUG -----
        angle_to_goal = math.atan2(self.goal_y - self.robot_y, self.goal_x - self.robot_x)
        
        # Encontrar a leitura do sensor na direção do objetivo
        goal_angle_in_lidar = self.normalize_angle(angle_to_goal - self.robot_theta)
        goal_lidar_index = int((goal_angle_in_lidar * 180 / math.pi) % 360)
        dist_in_goal_direction = self.lidar_data[goal_lidar_index]

        # Estado: Mover-se para o objetivo
        if self.state == "GO_TO_GOAL":
            if dist_in_goal_direction < self.dist_to_wall + 0.2: # Obstáculo no caminho
                self.state = "BOUNDARY_FOLLOWING"
                self.hit_point = (self.robot_x, self.robot_y)
                self.d_min = dist_to_goal
                self.get_logger().info("🚧 Obstáculo no caminho! Mudando para BOUNDARY_FOLLOWING.")
            else: # Caminho livre
                angle_diff = self.normalize_angle(angle_to_goal - self.robot_theta)
                linear_vel = min(self.kp_linear * dist_to_goal, self.max_linear_vel)
                angular_vel = max(min(self.kp_angular * angle_diff, self.max_angular_vel), -self.max_angular_vel)
                twist_msg.linear.x = linear_vel
                twist_msg.angular.z = angular_vel

        # Estado: Seguir o contorno
        elif self.state == "BOUNDARY_FOLLOWING":
            # Atualizar a menor distância alcançada ao objetivo
            if dist_to_goal < self.d_min:
                self.d_min = dist_to_goal

            # Condição de saída: Se o robô pode ir para o objetivo e está mais perto do que antes
            dist_from_hit_point = math.sqrt((self.robot_x - self.hit_point[0])**2 + (self.robot_y - self.hit_point[1])**2)
            if dist_from_hit_point > 0.5 and dist_to_goal < self.d_min - 0.1: # Heurística para sair
                 if dist_in_goal_direction > self.dist_to_wall + 0.5:
                    self.state = "GO_TO_GOAL"
                    self.get_logger().info("🎯 Caminho livre e mais perto do objetivo! Mudando para GO_TO_GOAL.")
                    return

            # Lógica de seguir a parede (tangente)
            # Encontrar o ponto de descontinuidade (borda do obstáculo)
            min_dist_front = float('inf')
            min_dist_angle = 0
            for i in range(180): # Analisar a frente do robô
                if self.lidar_data[i] < min_dist_front:
                    min_dist_front = self.lidar_data[i]
                    min_dist_angle = i

            for i in range(181, 360): # Analisar a frente do robô
                if self.lidar_data[i] < min_dist_front:
                    min_dist_front = self.lidar_data[i]
                    min_dist_angle = i - 360

            angle_to_obstacle = min_dist_angle * math.pi / 180.0
            
            # Seguir a parede mantendo uma distância
            if min_dist_front < self.dist_to_wall:
                # Muito perto, afastar-se
                twist_msg.linear.x = 0.05
                twist_msg.angular.z = -self.max_angular_vel * 0.8
            else:
                # Virar em direção ao obstáculo para não o perder
                twist_msg.linear.x = self.max_linear_vel * 0.7
                twist_msg.angular.z = self.kp_angular * angle_to_obstacle

        self.publisher_.publish(twist_msg)
        state_msg = String()
        state_msg.data = self.state
        self.state_publisher_.publish(state_msg)
        
    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

def main(args=None):
    rclpy.init(args=args)
    tangent_bug_node = TangentBugNode()
    try:
        rclpy.spin(tangent_bug_node)
    except KeyboardInterrupt:
        pass
    finally:
        tangent_bug_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
