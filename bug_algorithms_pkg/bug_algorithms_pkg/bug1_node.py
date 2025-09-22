import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import math
import numpy as np

# Importação explícita das políticas de Qualidade de Serviço (QoS)
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

class Bug1Node(Node):
    def __init__(self):
        super().__init__('bug1_node')

        # Criação de perfis de QoS diferentes para diferentes tipos de dados
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,  # Mudado para BEST_EFFORT para sensores
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5  # Aumentado para manter mais mensagens
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
        self.dist_to_wall = 0.8
        self.kp_linear = 0.5
        self.kp_angular = 0.6

        # Variáveis da máquina de estados
        self.state = "GO_TO_GOAL"
        self.leave_point_x = 0.0
        self.leave_point_y = 0.0

        # Inicializar variáveis de posição e sensores
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0
        self.lidar_data = []
        
        # Flags para verificar se recebemos dados
        self.odom_received = False
        self.scan_received = False

        # Publishers
        self.publisher_ = self.create_publisher(TwistStamped, 'cmd_vel', reliable_qos)
        self.state_publisher_ = self.create_publisher(String, 'bug1_state', reliable_qos)
        
        # Subscribers com QoS apropriado para sensores (BEST_EFFORT)
        self.subscription_scan = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            sensor_qos)

        self.subscription_odom = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            sensor_qos)
        
        # Timer para controle
        self.timer_ = self.create_timer(0.1, self.timer_callback)
        
        # Timer para debug - verifica se os dados estão chegando
        self.debug_timer_ = self.create_timer(2.0, self.debug_callback)
        
        self.get_logger().info('Nó Bug1 iniciado com sucesso.')
        self.get_logger().info(f'Aguardando dados dos sensores nos tópicos /scan e /odom...')
    
    def debug_callback(self):
        """Callback de debug para verificar status dos sensores"""
        if not self.odom_received and not self.scan_received:
            self.get_logger().warn("Nenhum dado de sensor recebido ainda. Verificando tópicos...")
            self.get_logger().info("💡 Execute o script de diagnóstico: ./quick_diagnostic.sh")
            self.get_logger().info("💡 Ou tente: ros2 topic list | grep -E '(scan|odom)'")
        elif not self.odom_received:
            self.get_logger().warn("Dados de odometria não recebidos. Verificando tópico /odom...")
            self.get_logger().info("💡 Teste: ros2 topic echo /odom --once")
        elif not self.scan_received:
            self.get_logger().warn("Dados de LaserScan não recebidos. Verificando tópico /scan...")
            self.get_logger().info("💡 Teste: ros2 topic echo /scan --once")
        else:
            self.get_logger().info("✅ Todos os sensores funcionando corretamente!")
            # Cancelar este timer de debug uma vez que tudo está funcionando
            if hasattr(self, 'debug_timer_'):
                self.debug_timer_.cancel()
    
    def odom_callback(self, msg):
        """Callback para processar dados de odometria"""
        try:
            self.robot_x = msg.pose.pose.position.x
            self.robot_y = msg.pose.pose.position.y
            self.robot_theta = self.quat_to_euler(msg.pose.pose.orientation)[2]
            
            if not self.odom_received:
                self.get_logger().info("Dados de odometria recebidos com sucesso!")
                self.odom_received = True
                
        except Exception as e:
            self.get_logger().error(f"Erro ao processar odometria: {e}")
        
    def scan_callback(self, msg):
        """Callback para processar dados do laser scan"""
        try:
            # Filtrar valores inválidos (inf, nan)
            ranges = np.array(msg.ranges)
            ranges[np.isinf(ranges)] = msg.range_max  # Substituir inf por range_max
            ranges[np.isnan(ranges)] = msg.range_max  # Substituir nan por range_max
            
            self.lidar_data = ranges.tolist()
            
            if not self.scan_received:
                self.get_logger().info(f"Dados de LaserScan recebidos! Total de {len(self.lidar_data)} leituras")
                self.scan_received = True
                
        except Exception as e:
            self.get_logger().error(f"Erro ao processar LaserScan: {e}")

    def quat_to_euler(self, quat):
        """Converte quaternion para ângulos de Euler"""
        try:
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
        except Exception as e:
            self.get_logger().error(f"Erro na conversão quaternion-euler: {e}")
            return 0.0, 0.0, 0.0

    def get_min_distance_front(self):
        """Calcula a distância mínima frontal do robô"""
        if not self.lidar_data or len(self.lidar_data) < 360:
            return float('inf')
        
        try:
            # Considerar um cone frontal mais amplo (±45 graus)
            front_indices = list(range(0, 45)) + list(range(315, 360))
            front_distances = [self.lidar_data[i] for i in front_indices if i < len(self.lidar_data)]
            
            if front_distances:
                return min(front_distances)
            else:
                return float('inf')
        except Exception as e:
            self.get_logger().error(f"Erro ao calcular distância frontal: {e}")
            return float('inf')

    def timer_callback(self):
        """Callback principal de controle"""
        twist_msg = TwistStamped()
        twist_msg.header.stamp = self.get_clock().now().to_msg()
        twist_msg.header.frame_id = "base_footprint"
        
        # Verificar se temos dados dos sensores
        if not self.odom_received or not self.scan_received:
            # Parar o robô se não temos dados
            twist_msg.twist.linear.x = 0.0
            twist_msg.twist.angular.z = 0.0
            self.publisher_.publish(twist_msg)
            return

        # Calcular distâncias
        min_dist_front = self.get_min_distance_front()
        dist_to_goal = math.sqrt((self.goal_x - self.robot_x)**2 + (self.goal_y - self.robot_y)**2)
        
        # Verificar se chegou ao objetivo
        if dist_to_goal < 0.2:
            self.get_logger().info("🎯 Objetivo alcançado!")
            twist_msg.twist.linear.x = 0.0
            twist_msg.twist.angular.z = 0.0
            self.publisher_.publish(twist_msg)
            return

        # Máquina de Estados do Bug1
        if self.state == "GO_TO_GOAL":
            if min_dist_front < self.dist_to_wall:
                self.get_logger().info("🚧 Obstáculo detectado! Mudando para BOUNDARY_FOLLOWING.")
                self.state = "BOUNDARY_FOLLOWING"
                self.leave_point_x = self.robot_x
                self.leave_point_y = self.robot_y
            else:
                # Navegar em direção ao objetivo
                angle_to_goal = math.atan2(self.goal_y - self.robot_y, self.goal_x - self.robot_x)
                angle_diff = angle_to_goal - self.robot_theta

                # Normalizar o ângulo
                while angle_diff > math.pi:
                    angle_diff -= 2 * math.pi
                while angle_diff < -math.pi:
                    angle_diff += 2 * math.pi

                # Limitar velocidades
                linear_vel = min(self.kp_linear * dist_to_goal, 0.3)
                angular_vel = max(min(self.kp_angular * angle_diff, 0.5), -0.5)
                
                twist_msg.twist.linear.x = linear_vel
                twist_msg.twist.angular.z = angular_vel
        
        elif self.state == "BOUNDARY_FOLLOWING":
            # Comportamento de seguir parede
            if min_dist_front < 0.4:
                # Muito perto da parede - girar para a direita
                twist_msg.twist.linear.x = 0.0
                twist_msg.twist.angular.z = -0.5
            else:
                # Seguir a parede
                twist_msg.twist.linear.x = 0.2
                twist_msg.twist.angular.z = 0.3

            # Verificar se podemos sair do modo boundary following
            dist_from_leave_point = math.sqrt(
                (self.robot_x - self.leave_point_x)**2 + 
                (self.robot_y - self.leave_point_y)**2
            )
            
            if dist_from_leave_point > 0.5:
                dist_to_goal_from_current = math.sqrt(
                    (self.goal_x - self.robot_x)**2 + 
                    (self.goal_y - self.robot_y)**2
                )
                dist_to_goal_from_leave = math.sqrt(
                    (self.goal_x - self.leave_point_x)**2 + 
                    (self.goal_y - self.leave_point_y)**2
                )
                
                if dist_to_goal_from_current < dist_to_goal_from_leave and min_dist_front > self.dist_to_wall:
                    self.get_logger().info("🎯 Caminho livre e mais perto do objetivo! Mudando para GO_TO_GOAL.")
                    self.state = "GO_TO_GOAL"

        # Publicar comandos
        self.publisher_.publish(twist_msg)
        
        # Publicar estado atual
        state_msg = String()
        state_msg.data = self.state
        self.state_publisher_.publish(state_msg)

def main(args=None):
    rclpy.init(args=args)
    bug1_node = Bug1Node()
    
    try:
        rclpy.spin(bug1_node)
    except KeyboardInterrupt:
        pass
    finally:
        bug1_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
