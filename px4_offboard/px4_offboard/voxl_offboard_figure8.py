#!/usr/bin/env python3
import rclpy
import math
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleStatus, VehicleOdometry

class OchoHibrido(Node):
    def __init__(self):
        super().__init__('voxl_offboard_figure8')
        
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

        # Configuración del Ocho Matemático Oficial
        self.rate = 20  # 20 Hz
        self.radius = 1.0
        self.cycle_s = 8.0
        self.target_altitude = -1.5 # Z negativo es hacia arriba
        self.steps = int(self.cycle_s * self.rate)
        self.path = []
        
        # Variables de control
        self.home_x, self.home_y, self.home_z = 0.0, 0.0, 0.0
        self.current_x, self.current_y, self.current_z = 0.0, 0.0, 0.0
        self.odom_received = False
        self.fase_vuelo = 0
        self.counter = 0
        self.path_index = 0
        
        # Temporizador maestro a 20Hz
        self.timer = self.create_timer(1.0 / self.rate, self.timer_callback)
        self.get_logger().info('Ocho Híbrido Iniciado: Esperando señal de cámara VIO...')

    def vehicle_status_callback(self, msg):
        pass

    def odom_callback(self, msg):
        self.current_x = msg.position[0]
        self.current_y = msg.position[1]
        self.current_z = msg.position[2]
        self.odom_received = True

    def precalcular_ocho(self):
        """Genera los puntos matemáticos exactos sumados al Home actual"""
        dt = 1.0 / self.rate
        dadt = (2.0 * math.pi) / self.cycle_s
        r = self.radius

        for i in range(self.steps):
            msg = TrajectorySetpoint()
            a = (-math.pi / 2.0) + i * (2.0 * math.pi / self.steps)
            c = math.cos(a)
            c2a = math.cos(2.0 * a)
            c4a = math.cos(4.0 * a)
            c2am3 = c2a - 3.0
            c2am3_cubed = c2am3 ** 3
            s = math.sin(a)
            cc = c * c
            ss = s * s
            sspo = ss + 1.0
            ssmo = ss - 1.0
            sspos = sspo * sspo

            # Coordenadas relativas
            dx = -(r * c * s) / sspo
            dy = (r * c) / sspo
            
            # MAGIA: Anclamos las matemáticas a nuestro punto de despegue real (con float forzado)
            msg.position = [float(self.home_x + dx), float(self.home_y + dy), float(self.target_altitude)]
            
            # Feed-forward para no perder inercia (con float forzado)
            msg.velocity = [float(dadt * r * (ss * ss + ss + ssmo * cc) / sspos), float(-dadt * r * s * (ss + 2.0 * cc + 1.0) / sspos), float(0.0)]
            msg.acceleration = [float(-dadt * dadt * 8.0 * r * s * c * ((3.0 * c2a) + 7.0) / c2am3_cubed), float(dadt * dadt * r * c * ((44.0 * c2a) + c4a - 21.0) / c2am3_cubed), float(0.0)]
            msg.yaw = float(math.atan2(msg.velocity[1], msg.velocity[0]))

            self.path.append(msg)

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.position, msg.timestamp = True, int(self.get_clock().now().nanoseconds / 1000)
        msg.velocity = msg.acceleration = msg.attitude = msg.body_rate = False
        self.offboard_control_mode_publisher_.publish(msg)

    def publish_vehicle_command(self, command, **params):
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = float(params.get("param1", 0.0))
        msg.param2 = float(params.get("param2", 0.0))
        msg.target_system = msg.target_component = msg.source_system = msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher_.publish(msg)

    def timer_callback(self):
        # 1. BLOQUEO DE SEGURIDAD
        if not self.odom_received:
            return

        # 2. SEÑAL DE VIDA
        self.publish_offboard_control_mode()

        if self.fase_vuelo == 0:
            self.home_x = self.current_x
            self.home_y = self.current_y
            self.home_z = self.current_z
            self.precalcular_ocho()
            
            self.get_logger().info('Cámara VIO detectada. Home fijado. Solicitando despegue...')
            self.fase_vuelo = 1

        elif self.fase_vuelo == 1:
            # Enviamos el punto EXACTO envuelto en float()
            msg = TrajectorySetpoint()
            msg.position = [float(self.home_x), float(self.home_y), float(self.current_z)]
            msg.yaw = float("nan")
            msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
            self.trajectory_setpoint_publisher_.publish(msg)

            self.counter += 1
            if self.counter == 20: 
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
                self.get_logger().info('¡Armando motores y subiendo a 1.5m!')
                self.fase_vuelo = 2
                self.counter = 0

        elif self.fase_vuelo == 2:
            # Subir a la altura objetivo
            msg = TrajectorySetpoint()
            msg.position = [float(self.home_x), float(self.home_y), float(self.target_altitude)]
            msg.yaw = float("nan")
            msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
            self.trajectory_setpoint_publisher_.publish(msg)

            self.counter += 1
            if self.counter >= 80: 
                self.get_logger().info('¡Iniciando el Ocho Matemático Oficial!')
                self.fase_vuelo = 3

        elif self.fase_vuelo == 3:
            # Bucle del infinito
            if self.path_index < len(self.path):
                msg = self.path[self.path_index]
                msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
                self.trajectory_setpoint_publisher_.publish(msg)
                self.path_index += 1
            else:
                self.path_index = 0

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(OchoHibrido())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
