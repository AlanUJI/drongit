import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleStatus, VehicleOdometry

class AterrizajeSuave(Node):
    def __init__(self):
        super().__init__('aterrizaje_suave')

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
        self.status_sub = self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status_v1', self.vehicle_status_callback, qos_profile)
        self.odom_sub = self.create_subscription(VehicleOdometry, '/fmu/out/vehicle_odometry', self.odom_callback, qos_profile)

        # Variables de estado
        self.nav_state = 0
        self.odom_received = False
        self.landing_sent = False
        
        # Coordenadas
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        
        self.target_x = 0.0
        self.target_y = 0.0
        self.target_z = 0.0

        # Bucle a 10Hz
        self.timer = self.create_timer(0.1, self.timer_callback)

    def vehicle_status_callback(self, msg):
        self.nav_state = msg.nav_state

    def odom_callback(self, msg):
        self.current_x = msg.position[0]
        self.current_y = msg.position[1]
        self.current_z = msg.position[2]
        self.odom_received = True

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
        msg.yaw = float("nan") # Ignoramos el yaw para que mantenga el actual
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

    def timer_callback(self):
        if not self.odom_received:
            return # Esperamos a tener señal de posición antes de hacer nada

        # Mientras el dron esté volando manual (no offboard), el target persigue al dron
        # Esto evita que el dron pegue un tirón brusco al entrar en offboard
        if self.nav_state != 14: # 14 es el nav_state de OFFBOARD
            self.target_x = self.current_x
            self.target_y = self.current_y
            self.target_z = self.current_z
            self.landing_sent = False

        # Bombardeamos con setpoints para que PX4 nos autorice a usar el interruptor
        self.publish_offboard_control_mode()
        self.publish_trajectory_setpoint()

        # Cuando tú le das al interruptor en el mando...
        if self.nav_state == 14 and not self.landing_sent:
            self.get_logger().info('¡Modo Offboard detectado desde el mando!')
            self.get_logger().info('Iniciando aterrizaje automático suave...')
            
            # Mandamos el comando nativo de aterrizaje
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
            self.landing_sent = True

def main(args=None):
    rclpy.init(args=args)
    nodo_aterrizaje = AterrizajeSuave()
    rclpy.spin(nodo_aterrizaje)
    nodo_aterrizaje.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
