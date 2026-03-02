#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleStatus, VehicleOdometry

class DespegueYMantener(Node):
    def __init__(self):
        super().__init__('despegue_50cm_persistente')

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

        # Estados de telemetría
        self.nav_state = 0
        self.arming_state = 0
        self.current_z = 0.0
        self.home_z = 0.0
        
        self.odom_received = False
        self.odom_quality = 0
        self.fase_vuelo = 0
        self.counter = 0
        
        # Parámetros de misión
        self.target_altitude = -0.50  # 50 cm arriba (NED)
        self.dt = 0.1                 # 10 Hz
        self.altitude_step = 0.01     # Subida de 10 cm/s
        self.setpoint_z = 0.0

        self.timer = self.create_timer(self.dt, self.timer_callback)
        self.get_logger().info('Nodo Persistente iniciado. Esperando OK de QVIO...')

    def vehicle_status_callback(self, msg):
        self.nav_state = msg.nav_state
        self.arming_state = msg.arming_state

    def odom_callback(self, msg):
        self.current_x = float(msg.position[0])
        self.current_y = float(msg.position[1])
        self.current_z = float(msg.position[2])
        
        try:
            self.odom_quality = msg.quality
        except AttributeError:
            self.odom_quality = 100 

        self.odom_received = True

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher_.publish(msg)

    def publish_trajectory_setpoint(self, x, y, z):
        msg = TrajectorySetpoint()
        msg.position = [float(x), float(y), float(z)]
        # Evitar que el dron gire sobre sí mismo
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

        # FASE 0: Captura de Home
        if self.fase_vuelo == 0:
            self.home_x, self.home_y, self.home_z = self.current_x, self.current_y, self.current_z
            self.setpoint_z = self.home_z
            self.fase_vuelo = 1
            self.get_logger().info(f'Home fijado en Z: {self.home_z:.2f}. Solicitando Offboard...')

        # FASE 1: Asegurar Modo Offboard
        elif self.fase_vuelo == 1:
            self.publish_trajectory_setpoint(self.home_x, self.home_y, self.home_z)
            if self.nav_state != 14:
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
            else:
                self.get_logger().info('Confirmado: Modo Offboard activo. Solicitando Armado...')
                self.fase_vuelo = 2

        # FASE 2: Asegurar Armado
        elif self.fase_vuelo == 2:
            self.publish_trajectory_setpoint(self.home_x, self.home_y, self.home_z)
            if self.arming_state != 2:
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
            else:
                self.get_logger().info('Confirmado: Motores armados. Iniciando validación acumulativa de VIO...')
                self.fase_vuelo = 3
                self.counter = 0

        # FASE 3: Estabilización ACUMULATIVA en suelo (Empujando hacia abajo)
        elif self.fase_vuelo == 3:
            # ENGAÑO: Le decimos que vaya 20 cm bajo el suelo para forzar el ralentí sin elevarse
            self.publish_trajectory_setpoint(self.home_x, self.home_y, self.home_z + 0.20)
            
            # Solo sumamos si es bueno, NO reiniciamos si es malo.
            if self.odom_quality >= 50:
                self.counter += 1
                if self.counter % 10 == 0:
                    self.get_logger().info(f'Acumulando... Calidad VIO: {self.odom_quality}% ({self.counter/10:.0f}/5 seg)')
            else:
                self.get_logger().debug(f'Pausa en VIO ({self.odom_quality}%). Esperando mejora...')

            # Despegamos cuando hayamos sumado 50 ciclos (5 segundos) en total
            if self.counter >= 50: 
                self.get_logger().info('¡Validación VIO exitosa! Iniciando ascenso a 50cm...')
                # Resetear el setpoint_z al nivel del suelo real antes de subir
                self.setpoint_z = self.home_z 
                self.fase_vuelo = 4

        # FASE 4: Ascenso suave y mantenimiento
        elif self.fase_vuelo == 4:
            target_z = self.home_z + self.target_altitude
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
