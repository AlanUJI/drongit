#!/usr/bin/env python3
import rclpy
import math
import threading
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from geometry_msgs.msg import PointStamped
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleOdometry

class NavegadorRViz(Node):
    def __init__(self):
        super().__init__('navegador_rviz_px4')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.offboard_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)

        self.odom_sub = self.create_subscription(VehicleOdometry, '/fmu/out/vehicle_odometry', self.odom_callback, qos_profile)
        self.click_sub = self.create_subscription(PointStamped, '/clicked_point', self.click_callback, 10)

        self.dt = 0.1
        self.timer = self.create_timer(self.dt, self.timer_callback)

        self.current_x, self.current_y, self.current_z, self.current_yaw = 0.0, 0.0, 0.0, 0.0
        self.odom_received = False

        self.sp_x, self.sp_y, self.sp_z, self.sp_yaw = 0.0, 0.0, 0.0, 0.0
        self.meta_x, self.meta_y = None, None
        self.en_movimiento = False

        self.velocidad = 0.15
        self.get_logger().info('🚀 Navegador (Modo Dron): Mapeo 1:1 Puro.')

    def odom_callback(self, msg):
        self.current_x, self.current_y, self.current_z = float(msg.position[0]), float(msg.position[1]), float(msg.position[2])
        q = msg.q
        if not math.isnan(q[0]):
            w, x, y, z = q[0], q[1], q[2], q[3]
            self.current_yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
        self.odom_received = True

    def click_callback(self, msg):
        if not self.odom_received or self.en_movimiento: return

        # MAPEO 1:1 PURO Y DURO
        objetivo_x = msg.point.x
        objetivo_y = msg.point.y

        print("\n" + "="*50)
        print(f"🖥️  RVIZ: X={msg.point.x:.2f}, Y={msg.point.y:.2f}")
        print(f"🤖 DRON: X={objetivo_x:.2f}, Y={objetivo_y:.2f}")
        print("="*50)

        def pedir_confirmacion():
            if input('⚠️ ¿Volar? (y/n): ').lower().strip() == 'y':
                self.meta_x, self.meta_y = objetivo_x, objetivo_y
                self.en_movimiento = True
            else: self.get_logger().info('❌ Parado.')

        threading.Thread(target=pedir_confirmacion).start()

    def timer_callback(self):
        if not self.odom_received: return
        if not self.en_movimiento:
            self.sp_x, self.sp_y, self.sp_z, self.sp_yaw = self.current_x, self.current_y, self.current_z, self.current_yaw

        offboard_msg = OffboardControlMode()
        offboard_msg.position = True
        offboard_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_pub.publish(offboard_msg)

        if self.en_movimiento and self.meta_x is not None:
            dist_x, dist_y = self.meta_x - self.sp_x, self.meta_y - self.sp_y
            distancia_restante = math.hypot(dist_x, dist_y)
            paso = self.velocidad * self.dt 
            if distancia_restante > 0.05:
                self.sp_x += (dist_x / distancia_restante) * paso
                self.sp_y += (dist_y / distancia_restante) * paso
            else:
                self.sp_x, self.sp_y, self.en_movimiento = self.meta_x, self.meta_y, False
                self.get_logger().info('✅ LLEGAMOS.')
        
        traj_msg = TrajectorySetpoint()
        traj_msg.position = [float(self.sp_x), float(self.sp_y), float(self.sp_z)]
        traj_msg.yaw = float(self.sp_yaw)
        traj_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_pub.publish(traj_msg)

def main(args=None):
    rclpy.init(args=args)
    node = NavegadorRViz()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
