import rclpy # libreria principal de ros 2 para python, nos permite crear nodos, temporizadores y comunicarnos
from rclpy.node import Node # importamos la clase base para poder construir nuestro propio nodo
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy # ajusta la conexion de red (udp) para que sea veloz y priorice el ultimo mensaje recibido
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleStatus, VehicleOdometry # el diccionario que hace que ros 2 y px4 hablen el mismo idioma
import time # libreria para manejar el tiempo (aunque aqui usamos el timer interno de ros 2)

class OffboardControl(Node): # creamos nuestro nodo central de mando
    def __init__(self): # funcion que inicializa todo al arrancar el programa
        super().__init__('offboard_control') # le ponemos el nombre offboard_control a nuestro nodo

        qos_profile = QoSProfile( # configuramos la comunicacion para no tener atascos de datos
            reliability=ReliabilityPolicy.BEST_EFFORT, # prefiere rapidez antes que confirmar que lleguen todos los paquetes
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST, # si se acumulan mensajes, que se quede solo con el mas nuevo
            depth=1
        )

        # publicadores: los megafonos del codigo para enviarle ordenes al dron
        self.offboard_control_mode_publisher_ = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile) # avisa al dron que controlaremos posicion
        self.trajectory_setpoint_publisher_ = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile) # le envia las coordenadas meta
        self.vehicle_command_publisher_ = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos_profile) # manda comandos maestros como armar o aterrizar

        # suscriptores: los oidos del codigo para escuchar la telemetria del dron
        self.status_sub = self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status_v1', self.vehicle_status_callback, qos_profile) # escucha el estado general
        self.odom_sub = self.create_subscription(VehicleOdometry, '/fmu/out/vehicle_odometry', self.odom_callback, qos_profile) # escucha el gps/camara para saber la posicion real

        # variables para guardar el ultimo estado recibido
        self.vehicle_status = VehicleStatus()
        
        # coordenadas reales (donde esta ahora) y objetivo (a donde va)
        self.current_x = 0.0 # guardara la x real
        self.current_y = 0.0 # guardara la y real
        self.current_z = 0.0 # guardara la z real (la altura, que es negativa hacia arriba)
        self.target_x = 0.0 # guardara la x donde encendimos el dron para no derrapar
        self.target_y = 0.0 # guardara la y donde encendimos el dron para no derrapar

        self.counter = 0 # contador para los 5 segundos de seguridad inicial
        self.hover_ticks = 0 # contador para los 3 segundos que se queda flotando en el aire
        self.fase_vuelo = 0 # maquina de estados: 0 es espera, 1 es subir, 2 es flotar, 3 es aterrizar

        self.timer = self.create_timer(0.1, self.timer_callback) # el corazon del programa: ejecuta timer_callback 10 veces cada segundo

    def vehicle_status_callback(self, msg): # se dispara solita cuando el dron nos manda su estado
        self.vehicle_status = msg # guardamos el estado en nuestra variable

    def odom_callback(self, msg): # se dispara solita cada vez que la camara recalcula donde esta
        # leemos todas las coordenadas en tiempo real y las guardamos
        self.current_x = msg.position[0]
        self.current_y = msg.position[1]
        self.current_z = msg.position[2]

    def publish_offboard_control_mode(self): # envia un latido obligatorio para que el dron no se asuste
        msg = OffboardControlMode()
        msg.position = True # le decimos que mandaremos coordenadas
        msg.velocity = False # apagamos el control por velocidad
        msg.acceleration = False # apagamos el control por aceleracion
        msg.attitude = False # apagamos el control por inclinacion
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000) # le pone la hora exacta para sincronizarse
        self.offboard_control_mode_publisher_.publish(msg)

    def publish_trajectory_setpoint(self): # empaqueta la coordenada a la que queremos viajar
        msg = TrajectorySetpoint()
        # usamos target_x y target_y en lugar de 0.0 absoluto para que el punto cero sea donde esta apoyado ahora mismo
        msg.position = [self.target_x, self.target_y, -0.5] # el objetivo final (medio metro hacia arriba)
        msg.yaw = 0.0 # yaw a 0 para que no gire sobre si mismo
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher_.publish(msg)

    def publish_vehicle_command(self, command, **params): # plantilla general para enviarle comandos fuertes (mavlink) al dron
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.target_system = 1 # id del sistema del dron
        msg.target_component = 1 # id del componente
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True # avisa que la orden viene de fuera (de nuestro ordenador)
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher_.publish(msg)

    def arm(self): # atajo para mandar la orden de encender motores
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info('armando motores')

    def engage_offboard_mode(self): # atajo para pedirle permiso al dron para entrar en modo autonomo
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info('modo offboard activado (despegando...)')

    def land(self): # atajo para mandarle la orden automatica de bajar
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info('meta alcanzada. aterrizando...')

    def timer_callback(self): # la inteligencia de la operacion, se ejecuta todo el rato (10 veces por segundo)
        self.publish_offboard_control_mode() # lanza el latido. si el dron no lo recibe en 0.5 segundos entra en failsafe
        self.publish_trajectory_setpoint() # manda la posicion objetivo sin parar

        if self.fase_vuelo == 0: # fase de espera en el suelo
            # mientras esperamos los 5 segundos, vamos copiando la posicion real al objetivo
            # esto arregla el problema de que el dron se vaya hacia la izquierda, lo ancla en su sitio
            self.target_x = self.current_x
            self.target_y = self.current_y

            if self.counter == 50: # 50 ticks de 0.1s equivalen a 5 segundos clavados
                self.engage_offboard_mode() # pedimos permiso
                self.arm() # encendemos
                self.fase_vuelo = 1 # cambiamos de estado para empezar a subir
            else:
                self.counter += 1 # sumamos 1 al reloj y seguimos esperando

        elif self.fase_vuelo == 1: # fase de subida
            # calcula cuanto espacio le falta para llegar al medio metro restando las alturas
            distancia_al_objetivo = abs(self.current_z - (-0.5))
            
            if distancia_al_objetivo < 0.1: # si le faltan menos de 10 centimetros damos la meta por buena
                self.get_logger().info('altura objetivo alcanzada (0.5m). manteniendo posicion...')
                self.fase_vuelo = 2 # pasamos a la fase de flotar
                
        elif self.fase_vuelo == 2: # fase de quedarse quieto (hover)
            if self.hover_ticks == 30: # 30 ticks equivalen a 3 segundos flotando para hacer la foto
                self.land() # mandamos bajar
                self.fase_vuelo = 3 # mision terminada, no hace nada mas
            else:
                self.hover_ticks += 1 # sumamos 1 al reloj de flotar

def main(args=None): # la puerta de entrada que arranca el tinglado
    rclpy.init(args=args) # arranca ros 2
    offboard_control = OffboardControl() # crea nuestra obra de arte
    rclpy.spin(offboard_control) # pone el codigo a dar vueltas infinitas para que el timer jamas se detenga
    offboard_control.destroy_node() # si cancelamos con el teclado, destruye el nodo de forma limpia
    rclpy.shutdown() # apaga ros 2

if __name__ == '__main__':
    main()