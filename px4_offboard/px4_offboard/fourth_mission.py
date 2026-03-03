#!/usr/bin/env python3
import rclpy
import math  
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleStatus, VehicleOdometry

import subprocess
import threading

class MisionInspeccionPrecisa(Node):
    def __init__(self):
        super().__init__('mision_inspeccion_precisa')

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
        self.sp_yaw = 0.0 
        
        # Variables de la geometría de inspección
        self.center_x = 0.0
        self.center_y = 0.0
        self.entry_x = 0.0
        self.entry_y = 0.0
        
        self.odom_received = False
        self.real_qvio_quality = 0   
        self.fase_vuelo = 0
        self.counter = 0
        self.print_counter = 0
        
        # Parámetros de la Misión
        self.dt = 0.1                 
        self.target_altitude = -1.0   # Altura de 1 metro
        self.altitude_step = 0.01     
        self.velocidad_horizontal = 0.30 
        self.velocidad_descenso = 0.2   
        self.velocidad_angular = 0.15  # Velocidad de la órbita
        self.radio_orbita = 0.50       # Queremos que la órbita sea de 50 cm
        
        self.orbit_angle = 0.0
        self.orbit_accumulated = 0.0

        self.qvio_thread = threading.Thread(target=self.qvio_monitor_worker, daemon=True)
        self.qvio_thread.start()

        self.timer = self.create_timer(self.dt, self.timer_callback)
        self.get_logger().info('Nodo Inspección Avanzado iniciado. Esperando OK de VIO...')

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
        msg.yaw = float(self.sp_yaw)
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

        self.publish_offboard_control_mode()

        # FASE 0: Captura de Home
        if self.fase_vuelo == 0:
            self.home_x, self.home_y, self.home_z = self.current_x, self.current_y, self.current_z
            self.home_yaw = self.current_yaw 
            
            self.sp_x, self.sp_y, self.sp_z = self.home_x, self.home_y, self.home_z
            self.sp_yaw = self.home_yaw
            self.fase_vuelo = 1
            self.get_logger().info('Home fijado. Solicitando Offboard...')

        # FASE 1: Asegurar Modo Offboard
        elif self.fase_vuelo == 1:
            if self.nav_state != 14:
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
            else:
                self.fase_vuelo = 2

        # FASE 2: Asegurar Armado
        elif self.fase_vuelo == 2:
            if self.arming_state != 2:
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
            else:
                self.fase_vuelo = 3
                self.counter = 0

        # FASE 3: Estabilización VIO en suelo
        elif self.fase_vuelo == 3:
            self.sp_z = self.home_z + 0.15 
            if self.real_qvio_quality >= 50:
                self.counter += 1
            if self.counter >= 30: 
                self.get_logger().info('¡Iniciando ascenso a 1 metro!')
                self.sp_z = self.home_z
                self.fase_vuelo = 4

        # FASE 4: Ascenso suave
        elif self.fase_vuelo == 4:
            target_z = self.home_z + self.target_altitude
            if self.sp_z > target_z:
                self.sp_z -= self.altitude_step 
            else:
                self.sp_z = target_z
                self.get_logger().info('¡1 metro de altura alcanzado! Calculando ruta al objeto...')
                self.fase_vuelo = 5
                self.counter = 0

        # FASE 5: Cálculo de la ruta de acercamiento
        elif self.fase_vuelo == 5:
            self.counter += 1
            if self.counter >= 30: 
                # El objeto está a 240 cm (2.4m) delante y 118 cm (1.18m) a la izquierda respecto al origen.
                offset_fwd = 2.40
                offset_left = 1.18
                
                # 1. Coordenadas absolutas del objeto (POI)
                self.center_x = self.home_x + (offset_fwd * math.cos(self.home_yaw)) + (offset_left * math.sin(self.home_yaw))
                self.center_y = self.home_y + (offset_fwd * math.sin(self.home_yaw)) - (offset_left * math.cos(self.home_yaw))
                
                # 2. Ángulo de la línea recta desde el origen hasta el objeto
                angle_home_to_poi = math.atan2(self.center_y - self.home_y, self.center_x - self.home_x)
                
                # 3. Punto de entrada (50 cm antes de llegar al objeto en esa misma línea)
                self.entry_x = self.center_x - (self.radio_orbita * math.cos(angle_home_to_poi))
                self.entry_y = self.center_y - (self.radio_orbita * math.sin(angle_home_to_poi))
                
                self.get_logger().info('Acercándose al objetivo (hasta quedar a 50cm)...')
                self.fase_vuelo = 6

        # FASE 6: Vuelo de aproximación al punto de entrada
        elif self.fase_vuelo == 6:
            dist_x = self.entry_x - self.sp_x
            dist_y = self.entry_y - self.sp_y
            distancia_restante = math.hypot(dist_x, dist_y)
            paso = self.velocidad_horizontal * self.dt

            if distancia_restante > 0.05:
                self.sp_x += (dist_x / distancia_restante) * paso
                self.sp_y += (dist_y / distancia_restante) * paso
                # Hacemos que el dron mire al objeto mientras se acerca a él
                self.sp_yaw = math.atan2(self.center_y - self.sp_y, self.center_x - self.sp_x)
            else:
                self.sp_x = self.entry_x 
                self.sp_y = self.entry_y
                self.get_logger().info('¡Punto de entrada alcanzado! Preparando órbita...')
                self.fase_vuelo = 7

        # FASE 7: Inicialización de la Órbita
        elif self.fase_vuelo == 7:
            # Calculamos en qué ángulo del círculo nos encontramos ahora mismo respecto al POI
            self.orbit_angle = math.atan2(self.sp_y - self.center_y, self.sp_x - self.center_x)
            self.orbit_accumulated = 0.0
            
            self.get_logger().info(f'Iniciando órbita de 360° a {self.radio_orbita}m de distancia...')
            self.fase_vuelo = 8

        # FASE 8: Ejecutar Órbita Continua sobre el POI
        elif self.fase_vuelo == 8:
            if self.orbit_accumulated < 2 * math.pi:
                incremento = self.velocidad_angular * self.dt
                self.orbit_angle += incremento
                self.orbit_accumulated += incremento
                
                self.sp_x = self.center_x + (self.radio_orbita * math.cos(self.orbit_angle))
                self.sp_y = self.center_y + (self.radio_orbita * math.sin(self.orbit_angle))
                
                self.sp_yaw = math.atan2(self.center_y - self.sp_y, self.center_x - self.sp_x)
                
            else:
                self.get_logger().info('¡Escaneo 360° completado! Iniciando RTH...')
                self.fase_vuelo = 9

        # FASE 9: RTH Horizontal (Regreso al origen 0,0)
        elif self.fase_vuelo == 9:
            dist_x = self.home_x - self.sp_x
            dist_y = self.home_y - self.sp_y
            distancia_total = math.hypot(dist_x, dist_y)

            paso = self.velocidad_horizontal * self.dt 

            if distancia_total > 0.1: 
                self.sp_x += (dist_x / distancia_total) * paso
                self.sp_y += (dist_y / distancia_total) * paso
                # Opcional: Que mire hacia casa mientras vuelve
                self.sp_yaw = math.atan2(self.home_y - self.sp_y, self.home_x - self.sp_x)
            else:
                self.get_logger().info('¡Origen alcanzado! Iniciando descenso vertical...')
                self.sp_x = self.home_x
                self.sp_y = self.home_y
                self.sp_yaw = self.home_yaw # Restauramos el rumbo inicial de despegue
                self.fase_vuelo = 10

        # FASE 10: Descenso Vertical a 10cm
        elif self.fase_vuelo == 10:
            target_z_descenso = self.home_z - 0.10 
            if self.sp_z < target_z_descenso: 
                self.sp_z += (self.velocidad_descenso * self.dt)
            if self.current_z >= target_z_descenso:
                self.sp_z = target_z_descenso
                self.trigger_auto_land()
                self.fase_vuelo = 11

        # FASE 11: Esperar desarmado
        elif self.fase_vuelo == 11:
            if self.arming_state != 2:
                self.get_logger().info('Misión de inspección finalizada a salvo.')
                self.fase_vuelo = 12 
            return 

        if self.fase_vuelo < 12:
            self.publish_trajectory_setpoint()

def main(args=None):
    rclpy.init(args=args)
    node = MisionInspeccionPrecisa()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
