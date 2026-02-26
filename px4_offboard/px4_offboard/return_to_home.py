#!/usr/bin/env python3
import rclpy
import math
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleStatus, VehicleOdometry

class ReturnToHomeYaterrizaje(Node):
    def __init__(self):
        super().__init__('rth_aterrizaje_suave')

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
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        
        # Estas variables guardarán el punto imaginario al que seguimos paso a paso
        self.sp_x = 0.0
        self.sp_y = 0.0
        self.sp_z = 0.0
        
        self.odom_received = False
        self.fase_rth = 0
        
        # --- PARÁMETROS AJUSTABLES DE VELOCIDAD ---
        self.velocidad_horizontal = 0.5 # metros por segundo (Vuelo de vuelta)
        self.velocidad_descenso = 0.2   # metros por segundo (Aterrizaje)
        self.dt = 0.1 # Temporizador a 10Hz (0.1 segundos)

        self.timer = self.create_timer(self.dt, self.timer_callback)
        self.get_logger().info('Nodo de Return To Home Lento + Auto Land listo.')

    def vehicle_status_callback(self, msg):
        self.nav_state = msg.nav_state

    def odom_callback(self, msg):
        self.current_x = float(msg.position[0])
        self.current_y = float(msg.position[1])
        self.current_z = float(msg.position[2])
        self.odom_received = True

    def trigger_auto_land(self):
        msg = VehicleCommand()
        msg.command = VehicleCommand.VEHICLE_CMD_NAV_LAND
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher_.publish(msg)
        self.get_logger().info('¡Centro alcanzado y descenso finalizado! Modo NAV_LAND activado.')

    def timer_callback(self):
        if not self.odom_received:
            return

        self.publish_offboard_control_mode()

        if self.nav_state != 14: # Si no estamos en Offboard
            # Mantenemos el punto objetivo igual a la posición actual para no pegar tirones
            self.sp_x = self.current_x
            self.sp_y = self.current_y
            self.sp_z = self.current_z
            self.fase_rth = 0
            return

        # FASE 0: Iniciar RTH
        if self.fase_rth == 0:
            self.get_logger().info('Iniciando Return To Home (Volviendo al origen 0.0, 0.0)...')
            self.sp_x = self.current_x
            self.sp_y = self.current_y
            self.sp_z = self.current_z
            self.fase_rth = 1

        # FASE 1: Vuelo horizontal lento hacia el 0,0
        elif self.fase_rth == 1:
            # Calculamos la distancia que nos queda hasta el 0,0
            dist_x = 0.0 - self.sp_x
            dist_y = 0.0 - self.sp_y
            distancia_total = math.hypot(dist_x, dist_y)

            paso = self.velocidad_horizontal * self.dt # Cuántos centímetros avanzamos en esta milésima de segundo

            if distancia_total > 0.1: # Si estamos a más de 10cm del centro
                self.sp_x += (dist_x / distancia_total) * paso
                self.sp_y += (dist_y / distancia_total) * paso
            else:
                self.get_logger().info('¡Origen alcanzado! Iniciando descenso vertical...')
                self.sp_x = 0.0
                self.sp_y = 0.0
                self.fase_rth = 2

        # FASE 2: Descenso vertical en el origen
        elif self.fase_rth == 2:
            if self.current_z < -0.30: # Recordamos que Z es negativo hacia arriba
                self.sp_z += (self.velocidad_descenso * self.dt)
            else:
                self.trigger_auto_land()
                self.fase_rth = 3

        # FASE 3: Aterrizaje final delegado a PX4
        elif self.fase_rth == 3:
            pass # Solo mantenemos el enlace vivo mandando coordenadas

        self.publish_trajectory_setpoint()

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher_.publish(msg)

    def publish_trajectory_setpoint(self):
        msg = TrajectorySetpoint()
        msg.position = [float(self.sp_x), float(self.sp_y), float(self.sp_z)]
        # El NaN en el yaw hace que el dron no gire sobre sí mismo, simplemente se desliza hacia atrás o de lado hasta el centro
        msg.yaw = float("nan") 
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(ReturnToHomeYaterrizaje())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
