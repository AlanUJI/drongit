#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleStatus, VehicleOdometry

class AterrizajePerfecto(Node):
    def __init__(self):
        super().__init__('aterrizaje_suave')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.offboard_control_mode_publisher_ = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_publisher_ = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_publisher_ = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos_profile)

        self.status_sub = self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)
        self.odom_sub = self.create_subscription(VehicleOdometry, '/fmu/out/vehicle_odometry', self.odom_callback, qos_profile)

        self.nav_state = 0
        # Variables para leer odometría
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        
        # Variables para congelar la posición y bajar vertical
        self.lock_x = 0.0
        self.lock_y = 0.0
        self.target_z = 0.0
        
        self.odom_received = False
        self.is_landing = False
        self.landed_confirmed = False

        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info('Nodo de aterrizaje vertical y modo NAV_LAND listo.')

    def vehicle_status_callback(self, msg):
        self.nav_state = msg.nav_state

    def odom_callback(self, msg):
        self.current_x = float(msg.position[0])
        self.current_y = float(msg.position[1])
        self.current_z = float(msg.position[2])
        self.odom_received = True

    def trigger_auto_land(self):
        """Activa el modo de aterrizaje interno de PX4 para el toque final"""
        msg = VehicleCommand()
        msg.command = VehicleCommand.VEHICLE_CMD_NAV_LAND
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher_.publish(msg)
        self.get_logger().info('¡Colchón de aire alcanzado! Modo NAV_LAND activado. PX4 asume el contacto y desarme.')

    def timer_callback(self):
        if not self.odom_received:
            return

        self.publish_offboard_control_mode()

        if self.nav_state != 14: # Si no estamos en Offboard
            # Actualizamos constantemente la altura y posición "congelada" para no pegar tirones al activarlo
            self.target_z = self.current_z
            self.lock_x = self.current_x
            self.lock_y = self.current_y
            self.is_landing = False
            return

        if not self.is_landing:
            self.get_logger().info('Iniciando descenso vertical en el sitio exacto...')
            self.is_landing = True
            # AL EMPEZAR A ATERRIZAR, BLOQUEAMOS LAS COORDENADAS X e Y ACTUALES
            self.lock_x = self.current_x
            self.lock_y = self.current_y

        # Si ya hemos mandado el comando final, no hacemos nada más
        if self.landed_confirmed:
            return 

        # --- LÓGICA DE DESCENSO ---
        
        # Descenso Offboard suave hasta estar a 10 centímetros del suelo
        if self.current_z < -0.10:
            self.target_z += 0.02  # Baja 20cm por segundo
            self.publish_trajectory_setpoint()
            
        # Cuando llegamos a la capa crítica (menos de 10 cm)
        else:
            self.trigger_auto_land()
            self.landed_confirmed = True

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher_.publish(msg)

    def publish_trajectory_setpoint(self):
        msg = TrajectorySetpoint()
        # MANTENEMOS X e Y CONGELADAS PARA QUE BAJE RECTO
        msg.position = [float(self.lock_x), float(self.lock_y), float(self.target_z)]
        msg.yaw = float("nan")
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(AterrizajePerfecto())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
