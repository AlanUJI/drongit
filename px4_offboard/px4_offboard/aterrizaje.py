#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleStatus, VehicleOdometry

class AterrizajeSuave(Node):
    def __init__(self):
        super().__init__('aterrizaje_suave')

        # Configuración de Calidad de Servicio (QoS) para PX4
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

        # Suscriptores (HEMOS CAMBIADO EL TÓPICO DE STATUS PARA EVITAR ERRORES)
        self.status_sub = self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)
        self.odom_sub = self.create_subscription(VehicleOdometry, '/fmu/out/vehicle_odometry', self.odom_callback, qos_profile)

        # Variables de estado
        self.nav_state = 0
        self.odom_received = False
        self.landing_sent = False
        self.log_counter = 0
        
        # Coordenadas actuales y objetivo
        self.current_x, self.current_y, self.current_z = 0.0, 0.0, 0.0
        self.target_x, self.target_y, self.target_z = 0.0, 0.0, 0.0

        # Bucle de control a 10Hz
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info('Nodo de aterrizaje suave iniciado. Esperando odometría...')

    def vehicle_status_callback(self, msg):
        self.nav_state = msg.nav_state

    def odom_callback(self, msg):
        self.current_x = msg.position[0]
        self.current_y = msg.position[1]
        self.current_z = msg.position[2]
        self.odom_received = True

    def timer_callback(self):
        if not self.odom_received:
            return

        # LOG DE DIAGNÓSTICO: Verás el estado de PX4 cada 2 segundos
        self.log_counter += 1
        if self.log_counter % 20 == 0:
            self.get_logger().info(f'Estado PX4: {self.nav_state} | Pos Z: {self.current_z:.2f}')

        # Enviamos señales de vida de Offboard constantemente
        self.publish_offboard_control_mode()
        self.publish_trajectory_setpoint()

        # Lógica de detección del mando
        if self.nav_state != 14:  # 14 = MODO OFFBOARD
            # El punto objetivo persigue al dron mientras vuelas tú
            self.target_x = self.current_x
            self.target_y = self.current_y
            self.target_z = self.current_z
            self.landing_sent = False
        else:
            # Si entramos en Offboard y no hemos mandado aterrizar todavía...
            if not self.landing_sent:
                self.get_logger().info('¡INTERRUPTOR OFFBOARD DETECTADO!')
                self.get_logger().info('Iniciando aterrizaje suave automático...')
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
                self.landing_sent = True

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.position, msg.timestamp = True, int(self.get_clock().now().nanoseconds / 1000)
        msg.velocity = msg.acceleration = msg.attitude = msg.body_rate = False
        self.offboard_control_mode_publisher_.publish(msg)

    def publish_trajectory_setpoint(self):
        msg = TrajectorySetpoint()
        msg.position = [float(self.target_x), float(self.target_y), float(self.target_z)]
        msg.yaw = float("nan")
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher_.publish(msg)

    def publish_vehicle_command(self, command, **params):
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.target_system = msg.target_component = msg.source_system = msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    nodo = AterrizajeSuave()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    nodo.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
