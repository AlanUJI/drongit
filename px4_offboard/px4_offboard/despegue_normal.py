#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleStatus, VehicleOdometry

class DespegueYMantener(Node):
    def __init__(self):
        super().__init__('despegue_50cm')

        # Configuración de Calidad de Servicio (QoS)
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

        # Suscriptores
        self.status_sub = self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)
        self.odom_sub = self.create_subscription(VehicleOdometry, '/fmu/out/vehicle_odometry', self.odom_callback, qos_profile)

        # Variables de estado interno
        self.nav_state = 0
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        
        # Referencias espaciales fijas (Home)
        self.home_x = 0.0
        self.home_y = 0.0
        self.home_z = 0.0
        
        # Lógica de la máquina de estados
        self.odom_received = False
        self.fase_vuelo = 0
        self.counter = 0
        
        # Parámetros de la misión
        self.target_altitude = -0.50  # 50 cm de altura
        self.dt = 0.1                 # Frecuencia a 10 Hz

        self.timer = self.create_timer(self.dt, self.timer_callback)
        self.get_logger().info('Nodo de despegue estático iniciado. Esperando odometría VIO...')

    def vehicle_status_callback(self, msg):
        self.nav_state = msg.nav_state

    def odom_callback(self, msg):
        self.current_x = float(msg.position[0])
        self.current_y = float(msg.position[1])
        self.current_z = float(msg.position[2])
        self.odom_received = True

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher_.publish(msg)

    def publish_trajectory_setpoint(self, x, y, z):
        msg = TrajectorySetpoint()
        msg.position = [float(x), float(y), float(z)]
        msg.yaw = float("nan")
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher_.publish(msg)

    def publish_vehicle_command(self, command, **params):
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = float(params.get("param1", 0.0))
        msg.param2 = float(params.get("param2", 0.0))
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher_.publish(msg)

    def timer_callback(self):
        if not self.odom_received:
            return

        # Señal de vida Offboard a >2Hz
        self.publish_offboard_control_mode()

        # FASE 0: Anclaje de odometría inicial
        if self.fase_vuelo == 0:
            self.home_x = self.current_x
            self.home_y = self.current_y
            self.home_z = self.current_z
            self.get_logger().info('Origen VIO referenciado. Solicitando armado en 1 segundo...')
            self.fase_vuelo = 1

        # FASE 1: Estabilización previa al armado
        elif self.fase_vuelo == 1:
            self.publish_trajectory_setpoint(self.home_x, self.home_y, self.current_z)
            self.counter += 1
            if self.counter >= int(1.0 / self.dt):
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
                self.get_logger().info('Motores armados. Ascendiendo a 50 cm y manteniendo posición indefinidamente.')
                self.fase_vuelo = 2

        # FASE 2: Ascenso y mantenimiento (Hovering continuo)
        elif self.fase_vuelo == 2:
            altitud_deseada = self.home_z + self.target_altitude
            self.publish_trajectory_setpoint(self.home_x, self.home_y, altitud_deseada)
            # Al no haber transición a una Fase 3, el dron se quedará aquí hasta que cortes el programa.

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(DespegueYMantener())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
