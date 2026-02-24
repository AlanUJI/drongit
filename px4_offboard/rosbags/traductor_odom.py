import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from px4_msgs.msg import VehicleOdometry
from nav_msgs.msg import Odometry

class TraductorOdom(Node):
    def __init__(self):
        super().__init__('traductor_odom')
        # ¡EL TRUCO ESTÁ AQUÍ! Añadimos qos_profile_sensor_data (Best Effort)
        self.sub = self.create_subscription(
            VehicleOdometry, 
            '/fmu/out/vehicle_odometry', 
            self.listener_callback, 
            qos_profile_sensor_data)
        
        # El publicador hacia RViz sí va en modo normal (Reliable)
        self.pub = self.create_publisher(Odometry, '/odom_rviz', 10)

    def listener_callback(self, msg):
        out = Odometry()
        out.header.frame_id = "rviz_world"
        out.child_frame_id = "base_link"
        
        # Le ponemos la hora del sistema para que RViz no lo descarte
        out.header.stamp = self.get_clock().now().to_msg()
        
        # Convertimos las coordenadas espaciales (NED a ENU)
        out.pose.pose.position.x = float(msg.position[1])  # Este
        out.pose.pose.position.y = float(msg.position[0])  # Norte
        out.pose.pose.position.z = float(-msg.position[2]) # Arriba
        
        self.pub.publish(out)
        print("Traduciendo frame de odometría...", end="\r")

def main():
    rclpy.init()
    node = TraductorOdom()
    print("Traductor funcionando en modo Best Effort. Enviando trayectoria a /odom_rviz...")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
