import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleStatus, VehicleOdometry
import time
import math # Librería matemática para la Lemniscata

class OffboardControl(Node):
    def __init__(self):
        super().__init__('offboard_control')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.offboard_control_mode_publisher_ = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_publisher_ = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_publisher_ = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos_profile)

        self.status_sub = self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status_v1', self.vehicle_status_callback, qos_profile)
        self.odom_sub = self.create_subscription(VehicleOdometry, '/fmu/out/vehicle_odometry', self.odom_callback, qos_profile)

        self.vehicle_status = VehicleStatus()
        
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        
        self.target_x = 0.0
        self.target_y = 0.0
        self.target_z = -1.0 # Altura de 1 metro
        self.target_yaw = 0.0

        # VARIABLES DEL INFINITO (Lemniscata de Bernoulli)
        self.escala = 1.5 # Tamaño del infinito (anchura total será aprox el doble de esto)
        self.velocidad_angular = 0.3 # Velocidad de recorrido (más bajo = más suave)
        self.tiempo_inicio = 0.0

        # CÁLCULO DE VUELTAS Y TIEMPO
        self.vueltas_deseadas = 10
        self.tiempo_por_vuelta = (2.0 * math.pi) / self.velocidad_angular 
        self.tiempo_total_vuelo = self.vueltas_deseadas * self.tiempo_por_vuelta

        self.counter = 0     
        self.fase_vuelo = 0  

        self.timer = self.create_timer(0.1, self.timer_callback)

    def vehicle_status_callback(self, msg):
        self.vehicle_status = msg

    def odom_callback(self, msg):
        self.current_x = msg.position[0]
        self.current_y = msg.position[1]
        self.current_z = msg.position[2]

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher_.publish(msg)

    def publish_trajectory_setpoint(self):
        msg = TrajectorySetpoint()
        msg.position = [float(self.target_x), float(self.target_y), float(self.target_z)] 
        msg.yaw = float(self.target_yaw)
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher_.publish(msg)

    def publish_vehicle_command(self, command, **params):
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher_.publish(msg)

    def arm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info('¡ARMANDO MOTORES!')

    def engage_offboard_mode(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info('¡MODO OFFBOARD ACTIVADO! (Iniciando Lemniscata de Bernoulli...)')

    def land(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info('¡10 VUELTAS AL INFINITO COMPLETADAS! Aterrizando...')

    def timer_callback(self):
        self.publish_offboard_control_mode()
        self.publish_trajectory_setpoint()

        if self.fase_vuelo == 0:
            if self.counter == 50:
                self.engage_offboard_mode()
                self.arm()
                self.tiempo_inicio = time.time() # Arrancamos cronómetro
                self.fase_vuelo = 1 
            else:
                self.counter += 1

        elif self.fase_vuelo == 1:
            tiempo_actual = time.time() - self.tiempo_inicio
            w = self.velocidad_angular
            
            # 1. Matemáticas para el INFINITO (Lemniscata)
            # El denominador es la clave para aplastar el centro y curvar los bordes
            denom = 1.0 + math.pow(math.cos(w * tiempo_actual), 2)
            
            # Asignamos el eje Y para que el dron dibuje el infinito de izquierda a derecha
            self.target_y = (self.escala * math.sin(w * tiempo_actual)) / denom
            self.target_x = (self.escala * math.sin(w * tiempo_actual) * math.cos(w * tiempo_actual)) / denom
            self.target_z = -1.0 
            
            # 2. Cálculo de la orientación (YAW) adelantándonos 0.1s en el tiempo
            # Esto evita derivadas complejas y hace que el dron encare las curvas perfectamente
            tiempo_futuro = tiempo_actual + 0.1
            denom_futuro = 1.0 + math.pow(math.cos(w * tiempo_futuro), 2)
            
            futuro_y = (self.escala * math.sin(w * tiempo_futuro)) / denom_futuro
            futuro_x = (self.escala * math.sin(w * tiempo_futuro) * math.cos(w * tiempo_futuro)) / denom_futuro
            
            # Hacemos que la "nariz" apunte siempre hacia ese punto futuro
            self.target_yaw = math.atan2(futuro_y - self.target_y, futuro_x - self.target_x)

            # 3. Aterrizar al cumplir el tiempo
            if tiempo_actual > self.tiempo_total_vuelo:
                self.fase_vuelo = 2 
                
        elif self.fase_vuelo == 2:
            self.land()
            self.fase_vuelo = 3

def main(args=None):
    rclpy.init(args=args)
    offboard_control = OffboardControl()
    rclpy.spin(offboard_control)
    offboard_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
