#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleStatus, VehicleOdometry

class AterrizajePerfecto(Node):
    def __init__(self):
        super().__init__('aterrizaje_suave')

        # CORRECCIÓN DE QoS: TRANSIENT_LOCAL va en Durability, no en Reliability
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
        self.current_z = 0.0
        self.target_z = 0.0
        self.odom_received = False
        self.is_landing = False
        self.landed_confirmed = False

        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info('Nodo de aterrizaje ultra-suave corregido y listo.')

    def vehicle_status_callback(self, msg):
        self.nav_state = msg.nav_state

    def odom_callback(self, msg):
        # En PX4, Z es negativo hacia arriba (NED)
        self.current_z = msg.position[2] 
        self.odom_received = True

    def disarm(self):
        # Comando para apagar motores a la fuerza
        msg = VehicleCommand()
        msg.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        msg.param1 = 0.0  # 0.0 es DISARM
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher_.publish(msg)
        self.get_logger().info('¡CONTACTO DETECTADO: APAGANDO MOTORES!')

    def timer_callback(self):
        if not self.odom_received:
            return

        self.publish_offboard_control_mode()

        if self.nav_state != 14:
            self.target_z = self.current_z
            self.is_landing = False
            return

        if not self.is_landing:
            self.get_logger().info('Descenso controlado activado...')
            self.is_landing = True

        # --- LÓGICA DE DESCENSO SEGÚN ALTURA ---
        
        # 1. Descenso Normal (si estamos a más de 40cm)
        if self.current_z < -0.40:
            self.target_z += 0.02  # Baja 20cm por segundo
        
        # 2. Descenso de precisión (entre 40cm y 10cm)
        elif self.current_z < -0.10:
            self.target_z += 0.005 # Baja solo 5cm por segundo (Súper suave)
            self.get_logger().info('Aproximación final suave (40cm)...', once=True)
        
        # 3. Apagado (menos de 10cm)
        else:
            if not self.landed_confirmed:
                self.disarm()
                self.landed_confirmed = True

        self.publish_trajectory_setpoint()

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher_.publish(msg)

    def publish_trajectory_setpoint(self):
        msg = TrajectorySetpoint()
        msg.position = [0.0, 0.0, float(self.target_z)]
        msg.yaw = float("nan")
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(AterrizajePerfecto())
    rclpy.shutdown()
