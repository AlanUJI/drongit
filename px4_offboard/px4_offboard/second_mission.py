#!/usr/bin/env python3
import rclpy
import math  
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleStatus, VehicleOdometry

import subprocess
import threading

class MisionCuadrado(Node):
    def __init__(self):
        super().__init__('mision_cuadrado_1m')

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
        self.arming_state = 0
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        self.current_yaw = 0.0 
        
        self.home_x = 0.0
        self.home_y = 0.0
        self.home_z = 0.0
        self.home_yaw = 0.0    
        
        self.sp_x = 0.0
        self.sp_y = 0.0
        self.sp_z = 0.0
        
        self.odom_received = False
        self.real_qvio_quality = 0   
        self.fase_vuelo = 0
        self.counter = 0
        self.print_counter = 0
        
        # <--- NUEVAS VARIABLES PARA RUTAS --->
        self.waypoints = []
        self.current_wp_index = 0
        
        # Parámetros de la Misión
        self.dt = 0.1                 
        self.target_altitude = -0.50  
        self.altitude_step = 0.01     
        self.velocidad_horizontal = 0.30 # Acelerado un poco a 30 cm/s para que el cuadrado no se haga eterno
        self.velocidad_descenso = 0.2   

        self.qvio_thread = threading.Thread(target=self.qvio_monitor_worker, daemon=True)
        self.qvio_thread.start()

        self.timer = self.create_timer(self.dt, self.timer_callback)
        self.get_logger().info('Nodo de Misión Cuadrado (1m) iniciado. Esperando OK de VIO...')

    def qvio_monitor_worker(self):
        try:
            process = subprocess.Popen(['voxl-inspect-qvio'], stdout=subprocess.PIPE, universal_newlines=True)
            for line in iter(process.stdout.readline, ''):
                if '|' in line and '%' in line:
                    try:
                        parts = line.split('|')
                        if len(parts) >= 6:
                            qual_str = parts[4].replace('%', '').strip()
                            self.real_qvio_quality = int(qual_str)
                    except ValueError:
                        pass
        except Exception as e:
            self.get_logger().error(f"Error en espía QVIO: {e}")

    def vehicle_status_callback(self, msg):
        self.nav_state = msg.nav_state
        self.arming_state = msg.arming_state

    def odom_callback(self, msg):
        self.current_x = float(msg.position[0])
        self.current_y = float(msg.position[1])
        self.current_z = float(msg.position[2])
        
        q = msg.q
        if not math.isnan(q[0]):
            w, x, y, z = q[0], q[1], q[2], q[3]
            self.current_yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
            
        self.odom_received = True

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher_.publish(msg)

    def publish_trajectory_setpoint(self):
        msg = TrajectorySetpoint()
        msg.position = [float(self.sp_x), float(self.sp_y), float(self.sp_z)]
        msg.yaw = float("nan") # Orientación fija, se mueve como un cangrejo lateralmente y hacia atrás
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

    def trigger_auto_land(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info('¡Modo NAV_LAND activado! Aterrizando...')

    def timer_callback(self):
        if not self.odom_received:
            return

        self.print_counter += 1
        if self.print_counter >= 10 and self.fase_vuelo < 8:
            # self.get_logger().info(f'[MONITOR VIO] Calidad REAL: {self.real_qvio_quality}%')
            self.print_counter = 0

        self.publish_offboard_control_mode()

        if self.fase_vuelo == 0:
            self.home_x, self.home_y, self.home_z = self.current_x, self.current_y, self.current_z
            self.home_yaw = self.current_yaw 
            
            self.sp_x, self.sp_y, self.sp_z = self.home_x, self.home_y, self.home_z
            self.fase_vuelo = 1
            self.get_logger().info('Home fijado. Solicitando Offboard...')

        elif self.fase_vuelo == 1:
            if self.nav_state != 14:
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
            else:
                self.fase_vuelo = 2

        elif self.fase_vuelo == 2:
            if self.arming_state != 2:
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
            else:
                self.fase_vuelo = 3
                self.counter = 0

        elif self.fase_vuelo == 3:
            self.sp_z = self.home_z + 0.15 
            if self.real_qvio_quality >= 50:
                self.counter += 1
            if self.counter >= 30: 
                self.get_logger().info('¡Ascenso a 50cm!')
                self.sp_z = self.home_z
                self.fase_vuelo = 4

        elif self.fase_vuelo == 4:
            target_z = self.home_z + self.target_altitude
            if self.sp_z > target_z:
                self.sp_z -= self.altitude_step 
            else:
                self.sp_z = target_z
                self.fase_vuelo = 5
                self.counter = 0

        # FASE 5: Vuelo estacionario (Hover) y Cálculo de las 4 Esquinas
        elif self.fase_vuelo == 5:
            self.counter += 1
            if self.counter >= 50: 
                self.get_logger().info('¡Calculando ruta en CUADRADO de 1x1 metros!')
                
                lado = 1.0 # 1 metro de lado
                
                # <--- CÁLCULO VECTORIAL DE LAS 4 ESQUINAS RELATIVAS AL MORRO --->
                # WP1: Delante (1m)
                wp1_x = self.home_x + (lado * math.cos(self.home_yaw))
                wp1_y = self.home_y + (lado * math.sin(self.home_yaw))
                
                # WP2: Derecha (Añadimos 90 grados = pi/2 radianes al ángulo original)
                wp2_x = wp1_x + (lado * math.cos(self.home_yaw + math.pi/2))
                wp2_y = wp1_y + (lado * math.sin(self.home_yaw + math.pi/2))
                
                # WP3: Atrás (Añadimos 180 grados = pi radianes al original)
                wp3_x = wp2_x + (lado * math.cos(self.home_yaw + math.pi))
                wp3_y = wp2_y + (lado * math.sin(self.home_yaw + math.pi))
                
                # WP4: Izquierda (Que matemáticamente es volver exactamente a casa)
                wp4_x = self.home_x
                wp4_y = self.home_y
                
                # Guardamos las esquinas en nuestra lista de Waypoints
                self.waypoints = [(wp1_x, wp1_y), (wp2_x, wp2_y), (wp3_x, wp3_y), (wp4_x, wp4_y)]
                self.current_wp_index = 0 # Empezamos por el wp1
                
                self.fase_vuelo = 6

        # FASE 6: NAVEGACIÓN POR WAYPOINTS (Ejecuta la matriz punto por punto)
        elif self.fase_vuelo == 6:
            # Si aún nos quedan puntos por visitar en la lista...
            if self.current_wp_index < len(self.waypoints):
                target_x, target_y = self.waypoints[self.current_wp_index]
                
                dist_x = target_x - self.sp_x
                dist_y = target_y - self.sp_y
                distancia_restante = math.hypot(dist_x, dist_y)
                paso = self.velocidad_horizontal * self.dt

                # Si no hemos llegado a la esquina, seguimos avanzando
                if distancia_restante > 0.05:
                    self.sp_x += (dist_x / distancia_restante) * paso
                    self.sp_y += (dist_y / distancia_restante) * paso
                else:
                    # Esquina alcanzada. Saltamos al siguiente objetivo de la lista.
                    self.sp_x = target_x 
                    self.sp_y = target_y
                    self.current_wp_index += 1
                    self.get_logger().info(f'¡Vértice {self.current_wp_index}/4 alcanzado!')
            
            # Si ya hemos visitado los 4 puntos...
            else:
                self.get_logger().info('¡Cuadrado completado perfectamente! Iniciando descenso...')
                self.fase_vuelo = 7

        # FASE 7: Descenso Vertical
        elif self.fase_vuelo == 7:
            target_z_descenso = self.home_z - 0.10 
            if self.sp_z < target_z_descenso: 
                self.sp_z += (self.velocidad_descenso * self.dt)
            if self.current_z >= target_z_descenso:
                self.sp_z = target_z_descenso
                self.trigger_auto_land()
                self.fase_vuelo = 8

        # FASE 8: Esperar desarmado final
        elif self.fase_vuelo == 8:
            if self.arming_state != 2:
                self.fase_vuelo = 9 
            return 

        if self.fase_vuelo < 9:
            self.publish_trajectory_setpoint()

def main(args=None):
    rclpy.init(args=args)
    node = MisionCuadrado()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
