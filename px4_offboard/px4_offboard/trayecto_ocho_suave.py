import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleOdometry

class OffboardFigureEight(Node):
    def __init__(self):
        super().__init__('offboard_ocho_suave')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Publicadores
        self.offboard_control_mode_publisher_ = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_publisher_ = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_publisher_ = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos_profile)
        
        # Suscriptor para saber dónde estamos al empezar
        self.odom_sub = self.create_subscription(VehicleOdometry, '/fmu/out/vehicle_odometry', self.odom_callback, qos_profile)

        # Variables de control
        self.start_time = None
        self.home_x = 0.0
        self.home_y = 0.0
        self.current_x = 0.0
        self.current_y = 0.0
        self.init_odom = False

        # Configuración del Ocho (Parámetros de ModalAI)
        self.radius = 1.0  # Radio del bucle en metros
        self.cycle_duration = 10.0  # Tiempo en segundos para completar un '8'
        self.target_alt = -1.5  # Altura de vuelo (Z negativo es arriba)

        self.timer = self.create_timer(0.05, self.timer_callback) # 20Hz para máxima suavidad
        self.phase = 0 # 0: Espera, 1: Despegue, 2: Ocho

    def odom_callback(self, msg):
        self.current_x = msg.position[0]
        self.current_y = msg.position[1]
        if not self.init_odom:
            self.home_x = msg.position[0]
            self.home_y = msg.position[1]
            self.init_odom = True

    def timer_callback(self):
        if not self.init_odom: return

        self.publish_offboard_control_mode()

        if self.phase == 0:
            # Fase de preparación: Enviamos comandos y armamos
            self.get_logger().info('Iniciando sistema... Despegando en 3s')
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
            self.start_time = self.get_clock().now()
            self.phase = 1

        curr_time_sec = (self.get_clock().now() - self.start_time).nanoseconds / 1e9

        if self.phase == 1:
            # Despegue suave a la altura objetivo
            self.publish_trajectory_setpoint(self.home_x, self.home_y, self.target_alt)
            if curr_time_sec > 5.0: # Tras 5 seg de despegue, empezamos el ocho
                self.phase = 2
                self.get_logger().info('Iniciando trayectoria en ocho fluida...')

        elif self.phase == 2:
            # LÓGICA MATEMÁTICA DEL OCHO (Lemniscata)
            t = curr_time_sec - 5.0 # Tiempo relativo al inicio del ocho
            omega = 2.0 * np.pi / self.cycle_duration
            
            # Fórmulas de la Lemniscata de Bernoulli
            # x = a * cos(t) / (1 + sin^2(t))
            # y = a * sin(t) * cos(t) / (1 + sin^2(t))
            scale = self.radius * np.sqrt(2)
            denom = 1 + (np.sin(omega * t) ** 2)
            
            dx = (scale * np.cos(omega * t)) / denom
            dy = (scale * np.sin(omega * t) * np.cos(omega * t)) / denom
            
            # Sumamos al punto de origen para que el 8 sea relativo a donde despegó
            target_x = self.home_x + dx
            target_y = self.home_y + dy
            
            # Calculamos el Yaw para que el dron mire hacia donde va (tangente)
            yaw = np.arctan2(dy, dx) 
            
            self.publish_trajectory_setpoint(target_x, target_y, self.target_alt, yaw)

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.position, msg.velocity, msg.timestamp = True, False, int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher_.publish(msg)

    def publish_trajectory_setpoint(self, x, y, z, yaw=0.0):
        msg = TrajectorySetpoint()
        msg.position = [float(x), float(y), float(z)]
        msg.yaw = float(yaw)
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher_.publish(msg)

    def publish_vehicle_command(self, command, **params):
        msg = VehicleCommand()
        msg.command, msg.target_system, msg.target_component = command, 1, 1
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.from_external, msg.timestamp = True, int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(OffboardFigureEight())
    rclpy.shutdown()
