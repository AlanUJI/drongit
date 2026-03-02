#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleStatus, VehicleOdometry

import subprocess
import threading

class DespegueAvanceAterrizaje(Node):
    def __init__(self):
        super().__init__('despegue_avance_aterrizaje')

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
        self.home_x = 0.0
        self.home_y = 0.0
        self.home_z = 0.0
        
        # Puntos de consigna iterativos
        self.sp_x = 0.0
        self.sp_y = 0.0
        self.sp_z = 0.0
        
        self.odom_received = False
        self.real_qvio_quality = 0   
        self.fase_vuelo = 0
        self.counter = 0
        self.print_counter = 0
        
        # Parámetros de la Misión
        self.dt = 0.1                 
        self.target_altitude = -0.50  
        self.altitude_step = 0.01     
        self.velocidad_horizontal = 0.20 # <--- NUEVO: 20 cm/s para avance suave
        self.velocidad_descenso = 0.2   

        # Hilo espía de calidad VIO
        self.qvio_thread = threading.Thread(target=self.qvio_monitor_worker, daemon=True)
        self.qvio_thread.start()

        self.timer = self.create_timer(self.dt, self.timer_callback)
        self.get_logger().info('Nodo de Despegue, Avance (0.5m) y Aterrizaje iniciado. Esperando OK de VIO...')

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
        self.odom_received = True

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher_.publish(msg)

    def publish_trajectory_setpoint(self):
        msg = TrajectorySetpoint()
        msg.position = [float(self.sp_x), float(self.sp_y), float(self.sp_z)]
        msg.yaw = float("nan") # Sin giros sobre su eje
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
        self.get_logger().info('¡Colchón de aire (10cm) alcanzado! Modo NAV_LAND activado. PX4 asume el toque final.')

    def timer_callback(self):
        if not self.odom_received:
            return

        self.print_counter += 1
        # Actualizado a < 8 porque hemos añadido una fase
        if self.print_counter >= 10 and self.fase_vuelo < 8:
            self.get_logger().info(f'[MONITOR VIO] Calidad REAL: {self.real_qvio_quality}%')
            self.print_counter = 0

        self.publish_offboard_control_mode()

        # FASE 0: Captura de Home
        if self.fase_vuelo == 0:
            self.home_x, self.home_y, self.home_z = self.current_x, self.current_y, self.current_z
            self.sp_x, self.sp_y, self.sp_z = self.home_x, self.home_y, self.home_z
            self.fase_vuelo = 1
            self.get_logger().info(f'Home fijado en Z: {self.home_z:.2f}. Solicitando Offboard...')

        # FASE 1: Asegurar Modo Offboard
        elif self.fase_vuelo == 1:
            if self.nav_state != 14:
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
            else:
                self.get_logger().info('Confirmado: Modo Offboard activo. Solicitando Armado...')
                self.fase_vuelo = 2

        # FASE 2: Asegurar Armado
        elif self.fase_vuelo == 2:
            if self.arming_state != 2:
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
            else:
                self.get_logger().info('Confirmado: Motores armados. Iniciando validación VIO en suelo...')
                self.fase_vuelo = 3
                self.counter = 0

        # FASE 3: Estabilización VIO de 3 segundos pegado al suelo
        elif self.fase_vuelo == 3:
            self.sp_z = self.home_z + 0.15 
            if self.real_qvio_quality >= 50:
                self.counter += 1
                if self.counter % 10 == 0:
                    self.get_logger().info(f'----> Acumulando progreso VIO: ({self.counter/10:.0f}/3 seg superados)')
            if self.counter >= 30: 
                self.get_logger().info('¡Validación VIO exitosa! Iniciando ascenso progresivo a 50cm...')
                self.sp_z = self.home_z
                self.fase_vuelo = 4

        # FASE 4: Ascenso suave
        elif self.fase_vuelo == 4:
            target_z = self.home_z + self.target_altitude
            if self.sp_z > target_z:
                self.sp_z -= self.altitude_step 
            else:
                self.sp_z = target_z
                self.get_logger().info('¡Altitud de 50cm alcanzada! Iniciando vuelo estacionario (5 segundos)...')
                self.fase_vuelo = 5
                self.counter = 0

        # FASE 5: Vuelo estacionario (Hover de 5 segundos)
        elif self.fase_vuelo == 5:
            self.counter += 1
            if self.counter >= 50: # 50 ciclos = 5 segundos
                self.get_logger().info('Hover finalizado. Iniciando avance horizontal de 0.5 metros...')
                self.fase_vuelo = 6

        # FASE 6: AVANCE HORIZONTAL (0.5 metros en el eje X)
        elif self.fase_vuelo == 6:
            target_x = self.home_x + 0.50 # Medio metro hacia adelante
            
            # Incrementamos progresivamente la X
            if self.sp_x < target_x:
                self.sp_x += (self.velocidad_horizontal * self.dt)
            else:
                self.sp_x = target_x # Aseguramos el valor exacto
                self.get_logger().info('¡Objetivo alcanzado! Iniciando descenso vertical en el sitio exacto...')
                # CONGELAMOS LA Y ACTUAL PARA NO DESVIARNOS AL BAJAR
                self.sp_y = self.current_y
                self.fase_vuelo = 7

        # FASE 7: Descenso Vertical en el sitio
        elif self.fase_vuelo == 7:
            target_z_descenso = self.home_z - 0.10 # Límite de 10 cm del suelo real
            
            # Bajamos el punto imaginario progresivamente
            if self.sp_z < target_z_descenso: 
                self.sp_z += (self.velocidad_descenso * self.dt)
            
            # Comprobamos si el dron FÍSICO ha cruzado la capa de los 10cm
            if self.current_z >= target_z_descenso:
                self.sp_z = target_z_descenso
                self.trigger_auto_land()
                self.fase_vuelo = 8

        # FASE 8: Esperar desarmado final
        elif self.fase_vuelo == 8:
            if self.arming_state != 2:
                self.get_logger().info('¡Motores desarmados! Aterrizaje completado con éxito.')
                self.fase_vuelo = 9 # Fin del ciclo
            
            # Interrumpimos la publicación de Trayectorias para no pelear con NAV_LAND
            return 

        # Siempre publicamos el setpoint al final del ciclo
        if self.fase_vuelo < 9:
            self.publish_trajectory_setpoint()

def main(args=None):
    rclpy.init(args=args)
    node = DespegueAvanceAterrizaje()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
