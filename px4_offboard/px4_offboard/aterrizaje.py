#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleStatus, VehicleOdometry
import time

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
        self.current_z = 0.0
        self.target_z = 0.0
        self.odom_received = False
        self.is_landing = False
        self.landed_confirmed = False

        # --- Variables para superar el Efecto Suelo ---
        self.stuck_timer_start = 0.0
        self.stuck_altitude_memory = 0.0
        self.stuck_tolerance = 0.05 # 5 centímetros de margen de movimiento

        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info('Nodo de aterrizaje anti-Efecto-Suelo listo.')

    def vehicle_status_callback(self, msg):
        self.nav_state = msg.nav_state

    def odom_callback(self, msg):
        self.current_z = float(msg.position[2]) 
        self.odom_received = True

    def disarm(self):
        msg = VehicleCommand()
        msg.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        msg.param1 = float(0.0)  
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher_.publish(msg)
        self.get_logger().info('¡CORTANDO MOTORES (Efecto Suelo Superado)!')

    def timer_callback(self):
        if not self.odom_received:
            return

        self.publish_offboard_control_mode()

        if self.nav_state != 14:
            self.target_z = self.current_z
            self.is_landing = False
            return

        if not self.is_landing:
            self.get_logger().info('Descenso activado...')
            self.is_landing = True
            self.stuck_timer_start = time.time()
            self.stuck_altitude_memory = self.current_z

        # --- LÓGICA DE DESCENSO SEGÚN ALTURA ---
        
        # 1. Descenso Normal (si estamos a más de 40cm)
        if self.current_z < -0.40:
            self.target_z += 0.02  # Baja 20cm por segundo
            self.stuck_timer_start = time.time() # Reseteamos timer porque baja bien
            self.stuck_altitude_memory = self.current_z
        
        # 2. Descenso de precisión (entre 40cm y 10cm)
        elif self.current_z < -0.10:
            self.target_z += 0.005 # Baja 5cm por segundo
            
            # --- DETECTOR DE "ATASCO" POR EFECTO SUELO ---
            # Si el dron apenas ha cambiado de altura respecto a nuestra memoria...
            if abs(self.current_z - self.stuck_altitude_memory) < self.stuck_tolerance:
                tiempo_atascado = time.time() - self.stuck_timer_start
                if tiempo_atascado > 3.0: # Si lleva 3 segundos atascado en la burbuja de aire
                    self.get_logger().warn('¡Efecto suelo detectado! Forzando aterrizaje y desarme.')
                    if not self.landed_confirmed:
                        self.disarm()
                        self.landed_confirmed = True
            else:
                # Si se ha movido más de 5cm, reseteamos el detector de atasco
                self.stuck_timer_start = time.time()
                self.stuck_altitude_memory = self.current_z
        
        # 3. Apagado natural (si logra superar el colchón y llega a menos de 10cm)
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
        msg.position = [float(0.0), float(0.0), float(self.target_z)]
        msg.yaw = float("nan")
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(AterrizajePerfecto())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
