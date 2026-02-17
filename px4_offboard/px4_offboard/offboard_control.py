import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleStatus, VehicleOdometry
import time

class OffboardControl(Node):
    def __init__(self):
        super().__init__('offboard_control')

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

        # Suscriptores (¡Aquí está la magia nueva!)
        self.status_sub = self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status_v1', self.vehicle_status_callback, qos_profile)
        self.odom_sub = self.create_subscription(VehicleOdometry, '/fmu/out/vehicle_odometry', self.odom_callback, qos_profile)

        # Variables de estado
        self.vehicle_status = VehicleStatus()
        self.current_z = 0.0 # Guardará la altura real
        self.counter = 0     # Solo para los 5 seg de seguridad iniciales
        self.hover_ticks = 0 # Para contar 3 seg una vez alcance la meta
        self.fase_vuelo = 0  # 0: Espera, 1: Subiendo, 2: En la meta, 3: Aterrizando

        # Timer (10Hz -> 0.1 segundos)
        self.timer = self.create_timer(0.1, self.timer_callback)

    def vehicle_status_callback(self, msg):
        self.vehicle_status = msg

    def odom_callback(self, msg):
        # Leemos la posición real en el eje Z (recuerda que hacia arriba es negativo)
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
        msg.position = [0.0, 0.0, -0.5] # Objetivo: 0.5m
        msg.yaw = 0.0 
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
        self.get_logger().info('¡MODO OFFBOARD ACTIVADO! (Despegando...)')

    def land(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info('¡META ALCANZADA! Aterrizando...')

    def timer_callback(self):
        self.publish_offboard_control_mode()
        self.publish_trajectory_setpoint()

        # MAQUINA DE ESTADOS INTELIGENTE
        if self.fase_vuelo == 0:
            # Esperamos 5 segundos por seguridad antes de hacer nada (50 ticks)
            if self.counter == 50:
                self.engage_offboard_mode()
                self.arm()
                self.fase_vuelo = 1 # Pasamos a la fase de subida
            else:
                self.counter += 1

        elif self.fase_vuelo == 1:
            # El dron está subiendo. Comprobamos si ya llegó a -0.5m (damos un margen de error de 10cm)
            distancia_al_objetivo = abs(self.current_z - (-0.5))
            
            if distancia_al_objetivo < 0.1:
                self.get_logger().info('¡Altura objetivo alcanzada (0.5m)! Manteniendo posición...')
                self.fase_vuelo = 2 # Pasamos a la fase de mantenerse quieto
                
        elif self.fase_vuelo == 2:
            # Nos quedamos quietos 3 segundos (30 ticks)
            if self.hover_ticks == 30:
                self.land()
                self.fase_vuelo = 3 # Terminamos
            else:
                self.hover_ticks += 1

def main(args=None):
    rclpy.init(args=args)
    offboard_control = OffboardControl()
    rclpy.spin(offboard_control)
    offboard_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()