import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from px4_msgs.msg import VehicleOdometry
from nav_msgs.msg import Odometry

class TraductorOdom(Node):
    def __init__(self):
        super().__init__('traductor_odom')
        # --- ODOMETRÍA PX4 (Músculos) ---
        self.sub = self.create_subscription(
            VehicleOdometry, 
            '/fmu/out/vehicle_odometry', 
            self.listener_callback, 
            qos_profile_sensor_data)
        
        self.pub = self.create_publisher(Odometry, '/odom_rviz', 10)

        # --- ODOMETRÍA VOXL (Cerebro) ---
        self.sub_voxl = self.create_subscription(
            Odometry,
            '/vvhub_body_wrt_fixed/odom',
            self.voxl_callback,
            qos_profile_sensor_data)

        self.pub_voxl = self.create_publisher(Odometry, '/odom_voxl_rviz', 10)

    def listener_callback(self, msg):
        out = Odometry()
        out.header.frame_id = "rviz_world"
        out.child_frame_id = "base_link"
        
        out.header.stamp = self.get_clock().now().to_msg()
        
        # Convertimos las coordenadas espaciales (NED a ENU)
        out.pose.pose.position.x = float(msg.position[1])  # Este
        out.pose.pose.position.y = float(msg.position[0])  # Norte
        out.pose.pose.position.z = float(-msg.position[2]) # Arriba
        
        self.pub.publish(out)
        print("Traduciendo frames de odometría (PX4 + VOXL)...", end="\r")

    def voxl_callback(self, msg):
        # Corregimos el frame_id que viene vacío en el rosbag para que RViz lo sitúe
        msg.header.frame_id = "rviz_world"
        msg.header.stamp = self.get_clock().now().to_msg()
        
        self.pub_voxl.publish(msg)

def main():
    rclpy.init()
    node = TraductorOdom()
    print("Traductor funcionando. Enviando PX4 a /odom_rviz y VOXL a /odom_voxl_rviz...")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
