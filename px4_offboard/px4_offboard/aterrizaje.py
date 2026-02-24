#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleStatus, VehicleOdometry

class AterrizajePerfecto(Node):
    def __init__(self):
        super().__init__('aterrizaje_perfecto')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=ReliabilityPolicy.TRANSIENT_LOCAL,
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
        self.get_logger().info('Nodo de aterrizaje ultra-suave listo.')

    def vehicle_status_callback(self, msg):
        self.nav_state = msg.nav_state

    def odom_callback(self, msg):
        self.current_z = msg.position[2] # Z es negativo hacia arriba
        self.odom_received = True

    def disarm(self):
        # Comando para apagar motores inmediatamente
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        self.get_logger().info('¡MOTORES APAGADOS!')

    def timer_callback(self):
        if not self.odom_received:
            return

        self.publish_offboard_control_mode()

        # Si no estamos en modo Offboard, el objetivo sigue al dron
        if self.nav_state != 14:
            self.target_z = self.current_z
            self.is_landing = False
            return

        # SI DETECTAMOS OFFBOARD (Estado 14)
        if not self.is_landing:
            self.get_logger().info('Iniciando descenso controlado...')
            self.is_landing = True

        # LÓGICA DE DESCENSO SUAVE
        # Si estamos a más de 40cm del suelo (Z < -0.4), bajamos a velocidad normal
        if self.current_z < -0.4:
            self.target_z += 0.03  # Baja 3cm cada 0.1s (30cm/s)
        # Si estamos a menos de 40cm, bajamos MUY lento
        elif self.current_z < -0.1:
            self.target_z += 0.01  # Baja 1cm cada 0.1s (10cm/s)
            self.get_logger().info('Zona de contacto: Reduciendo velocidad...', once=True)
        # Si estamos a menos de 10cm, consideramos que hemos tocado suelo
        else:
            if not landed_confirmed:
                self.disarm()
                self.landed_confirmed = True

        self.publish_trajectory_setpoint()

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.position, msg.timestamp = True, int(self.get_clock().now().nanoseconds / 1000)
        msg.velocity = msg.acceleration = msg.attitude = msg.body_rate = False
        self.offboard_control_mode_publisher_.publish(msg)

    def publish_trajectory_setpoint(self):
        msg = TrajectorySetpoint()
        msg.position = [0.0, 0.0, float(self.target_z)] # Mantenemos X e Y en 0 relativo
        msg.yaw = float("nan")
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher_.publish(msg)

    def publish_vehicle_command(self, command, **params):
        msg = VehicleCommand()
        msg.command, msg.target_system, msg.target_component = command, 1, 1
        msg.source_system, msg.source_component = 1, 1
        msg.param1 = params.get("param1", 0.0)
        msg.from_external, msg.timestamp = True, int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(AterrizajePerfecto())
    rclpy.shutdown()
