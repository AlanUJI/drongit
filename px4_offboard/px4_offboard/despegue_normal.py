#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleStatus, VehicleOdometry

class DespegueYMantener(Node):
    def __init__(self):
        super().__init__('despegue_50cm_suave')

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

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        self.home_x = 0.0
        self.home_y = 0.0
        self.home_z = 0.0
        
        self.odom_received = False
        self.fase_vuelo = 0
        self.counter = 0
        
        # Parámetros ajustados
        self.target_altitude = -0.50  # 50 cm
        self.dt = 0.1                 # 10 Hz
        self.altitude_step = 0.01     # Incremento por cada ciclo (10 cm por segundo)

        self.timer = self.create_timer(self.dt, self.timer_callback)
        self.get_logger().info('Nodo iniciado: Despegue con estabilización previa en suelo.')

    def vehicle_status_callback(self, msg):
        self.nav_state = msg.nav_state

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

    def publish_trajectory_setpoint(self, x, y, z):
        msg = TrajectorySetpoint()
        msg.position = [float(x), float(y), float(z)]
        msg.yaw = float("nan")
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

    def timer_callback(self):
        if not self.odom_received:
            return

        self.publish_offboard_control_mode()

        # FASE 0: Referenciar Home
        if self.fase_vuelo == 0:
            self.home_x, self.home_y, self.home_z = self.current_x, self.current_y, self.current_z
            self.setpoint_z = self.home_z
            self.fase_vuelo = 1
            self.counter = 0

        # FASE 1: Armado y cambio de modo
        elif self.fase_vuelo == 1:
            self.publish_trajectory_setpoint(self.home_x, self.home_y, self.home_z)
            self.counter += 1
            if self.counter == 10: # Esperar 1s enviando setpoints al suelo
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
                self.get_logger().info('Motores armados en ralentí.')
                self.fase_vuelo = 2
                self.counter = 0

        # FASE 2: Estabilización en suelo (5 segundos girando sin despegar)
        elif self.fase_vuelo == 2:
            self.publish_trajectory_setpoint(self.home_x, self.home_y, self.home_z)
            self.counter += 1
            if self.counter >= 50: # 5 segundos (50 * 0.1s)
                self.get_logger().info('Estabilización completada. Iniciando ascenso progresivo...')
                self.fase_vuelo = 3

        # FASE 3: Ascenso lento y mantenimiento
        elif self.fase_vuelo == 3:
            target_z = self.home_z + self.target_altitude
            
            # Si aún no hemos llegado a la altura deseada, subimos poco a poco
            if self.setpoint_z > target_z:
                self.setpoint_z -= self.altitude_step
            else:
                self.setpoint_z = target_z
            
            self.publish_trajectory_setpoint(self.home_x, self.home_y, self.setpoint_z)

def main(args=None):
    rclpy.init(args=args)
    node = DespegueYMantener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
