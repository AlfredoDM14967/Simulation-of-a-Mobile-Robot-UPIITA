import threading
from kivy.app import App
from kivymd.app import MDApp
from kivy.uix.boxlayout import BoxLayout
from kivy.clock import Clock
from kivy.properties import StringProperty
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64MultiArray
from kivy.lang import Builder
from kivy.clock import Clock
from nav_msgs.msg import Odometry
from kivy.graphics.texture import Texture
from kivy.uix.image import Image
from sensor_msgs.msg import NavSatFix, Imu
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
import tf_transformations

from time import sleep
from math import atan2

import cv2
import socket
import struct
import pickle

# Nodo de ROS2
class ReceptorRover(Node):
    def __init__(self):
        super().__init__('string_subscriber')
        self.MensajesRover = self.create_subscription(Odometry,"ttrobot_controller/odom_kalman",  self.listenerCallback,10)
        self.GPS_sub_ = self.create_subscription(NavSatFix, '/gps/fix',self.posfCallback, 10)
        self.GPS_sub_ = self.create_subscription(NavSatFix, '/gps/fix',self.posfCallback, 10)
        self.imu_pub_ = self.create_subscription(Imu, "/imu/out", self.ImuCallback, qos_profile=qos_profile_sensor_data)
        self.joint_sub_ = self.create_subscription(JointState, "joint_states", self.jointCallback, 10)

        self.InstRover = self.create_publisher(String, "Comandos", 10)
        self.SubInst_ = self.create_subscription(String, "Comandos", self.CommCallback, 10)
        self.Odom_ = Odometry()
        self.pos_gps_ = NavSatFix()
        self.Joint_ = JointState()
        self.Joint_.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.Joint_.velocity = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.pos_gps_.altitude = 0.0
        self.pos_gps_.latitude = 0.0
        self.Imu_ = Imu()
        self.v_ = '0.0'
        self.z = 0.0
        self.Tray_Fin_ = False
        self.IpCallback()

        #publicador de velocidad y direccion
        self.wheel_cmd_pub_ = self.create_publisher(Float64MultiArray, "simple_velocity_controller/commands", 10)
        self.dir_cmd_pos_pub_ = self.create_publisher(Float64MultiArray, "simple_position_controller/commands", 10)

        #Mensajes para velocidad y direccion
        self.wheel_speed_msg = Float64MultiArray()
        self.wheel_pos_msg = Float64MultiArray()

        #publicador del topico /Position
        self.PosD_pub_ = self.create_publisher(String, "/Position", 10)

    def listenerCallback(self, msg):
        self.Odom_ = msg
    
    def posfCallback(self, msg):
        self.pos_gps_ = msg
    
    def ImuCallback(self, Imu):
        self.Imu_ = Imu
    
    def CommCallback(self, msg):
        aux = str(msg.data).split()
        if aux[0] == 'v':
            self.v_ = aux[1]
        elif aux[0] == 'F':
            self.Tray_Fin_ = True
            
    def jointCallback(self, joint):
        self.Joint_ = joint
    
    def IpCallback(self):
        # Función para obtener la dirección IP
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(("8.8.8.8", 80))
            self.ip = s.getsockname()[0]
            s.close()
            IP = String()
            IP.data = 'I' + str(self.ip)
            self.InstRover.publish(IP)
        except:
            self.ip = 'No IP'
            self.get_logger().info("IP NO OBTENIDA")

class ROS2KivyApp(MDApp):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.ros_node = None
        self.vel_wheel = 0.0
        self.Modo = String()
        self.Modo.data = 'M'
        self.wheel_separation_ = 1.115
        self.wheel_radius_ = 0.105
        self.L1_ = 0.7
        self.screen = Builder.load_file('/ros_ws/TT/src/nodosros2/kv/interfaz.kv')

    def build(self):
        self.title = "Robot Movil App"

        # # Configuracion de la red del video
        # self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # self.client_socket.connect(('192.168.0.227', 8000))  # Cambia por la IP de tu Raspberry Pi
        # self.data = b""
        # self.payload_size = struct.calcsize("L")  # Para Python 2, usa "Q"
        # Clock.schedule_interval(self.updateVid, 1.0 / 30.0)
        
        Clock.schedule_interval(self.updatePos, 1.0 / 5.0)
        
        return self.screen
    
    def updateVid(self, dt):
        # Recibir los datos del socket
        while len(self.data) < self.payload_size:
            packet = self.client_socket.recv(4096)
            if not packet:
                break
            self.data += packet

        # Obtener el tamano del frame
        packed_size = self.data[:self.payload_size]
        self.data = self.data[self.payload_size:]
        frame_size = struct.unpack("L", packed_size)[0]  # Tamano del frame

        # Recibir el frame completo
        while len(self.data) < frame_size:
            packet = self.client_socket.recv(4096)
            if not packet:
                break
            self.data += packet

        # Extraer los datos del frame
        frame_data = self.data[:frame_size]
        self.data = self.data[frame_size:]

        # Deserializar el frame comprimido
        buffer = pickle.loads(frame_data)

        # Decodificar el frame JPEG
        frame = cv2.imdecode(buffer, cv2.IMREAD_COLOR)    
        # Convierte el frame en textura para Kivy
        buf = frame.tostring()
        texture = Texture.create(size=(frame.shape[1], frame.shape[0]), colorfmt='rgb')
        texture.blit_buffer(buf, colorfmt='rgb', bufferfmt='ubyte')
        texture.flip_vertical()

        # Actualiza la textura en el widget de la imagen
        self.screen.ids.video_feed.texture = texture
    
    def updatePos(self, *args):
        self.screen.ids.Pos_AcLat.text = "Lat: " + str(self.ros_node.pos_gps_.latitude)
        self.screen.ids.Pos_AcLon.text = "Lon: " + str(self.ros_node.pos_gps_.longitude)
        self.screen.ids.v_label.text = "Voltaje: " + self.ros_node.v_
        aux = (self.ros_node.Joint_.velocity[0] + self.ros_node.Joint_.velocity[1] + self.ros_node.Joint_.velocity[2])/3
        self.screen.ids.wl_label.text = "WL: " + str(aux)
        aux = (self.ros_node.Joint_.velocity[3] + self.ros_node.Joint_.velocity[4] + self.ros_node.Joint_.velocity[5])/3
        self.screen.ids.wr_label.text = "WR: " + str(aux)
        msg = String()
        if float(self.ros_node.v_) <= 22.0:
            msg.data = 'C4'
            self.ros_node.InstRover.publish(msg)
        if self.Modo.data == 'M':
            ang = self.ros_node.Imu_.orientation._z * 180 / 3.1415925
            self.screen.ids.Or_Ac.text = "Orientacion: " + str(ang)
        elif self.Modo.data == 'A':
            ang = self.ros_node.Imu_.orientation._z * 180 / 3.1415925
            self.screen.ids.Or_Ac.text = "Orientacion: " + str(ang)
            self.screen.ids.velocidad_label.text = "Velocidad: " + str(self.ros_node.Odom_.twist.twist.linear.x) + " m/s"
        if self.ros_node.Tray_Fin_:
            self.ros_node.Tray_Fin_ = False
            msg.data = 'C2'
            self.ros_node.InstRover.publish(msg)
        msg.data = 'H'
        self.ros_node.InstRover.publish(msg)

    ######################### Eventos de los botones de la interfaz ########################
    def SetVel(self):
        self.vel_wheel = self.screen.ids.velocidad_m.value * 3.979 / 100
    
    def SetMode(self, modo):
        self.Modo.data = modo
        if modo == 'A':
            msg_posd_ = String()
            msg_posd_.data = self.screen.ids.Lon_F.text + " " + self.screen.ids.Lat_F.text
            self.ros_node.PosD_pub_.publish(msg_posd_)
            self.Modo.data = 'C1'
            self.ros_node.InstRover.publish(self.Modo)
            self.Modo.data = 'A'
        elif modo == 'D':
            self.Modo.data = 'M'
            self.Avanzar_rover()
            sleep(5)
            self.Retroceder_rover()
            sleep(5)
            self.Derecha_rover()
            sleep(5)
            self.Izquierda_rover()
            sleep(5)
            self.Detener_rover()
            sleep(5)
            self.GiroD_rover()
            sleep(5)
            self.GiroI_rover()
            sleep(5)
            self.Detener_rover()
            msg = String()
            msg.data = 'C9'
            self.ros_node.InstRover.publish(self.Modo)
            sleep(9)
            self.Modo.data = 'M'

        self.ros_node.InstRover.publish(self.Modo)

    def Avanzar_rover(self, *args):
        if self.Modo.data == 'M':
            # self.ros_node.get_logger().info("Publishing at %d Hz" % self.vel_wheel)
            self.ros_node.wheel_speed_msg.data = [-self.vel_wheel, -self.vel_wheel, -self.vel_wheel,       # Velocidades de las ruedas del robot
                                                    self.vel_wheel, self.vel_wheel, self.vel_wheel]
            self.ros_node.wheel_pos_msg.data = [0.0,0.0,0.0,0.0]               # Angulos de las ruedas del robot
            self.ros_node.dir_cmd_pos_pub_.publish(self.ros_node.wheel_pos_msg)
            sleep(1)
            self.ros_node.wheel_cmd_pub_.publish(self.ros_node.wheel_speed_msg)
            msg = String()
            msg.data = 'C1'
            self.ros_node.InstRover.publish(msg)

    def Detener_rover(self, *args):
        if self.Modo.data == "M":
            self.ros_node.wheel_speed_msg.data = [0.0, 0.0, 0.0,       # Velocidades de las ruedas del robot
                                                    0.0, 0.0, 0.0]
            self.ros_node.wheel_pos_msg.data = [0.0,0.0,0.0,0.0]                # Angulos de las ruedas del robot
            self.ros_node.dir_cmd_pos_pub_.publish(self.ros_node.wheel_pos_msg)
            self.ros_node.wheel_cmd_pub_.publish(self.ros_node.wheel_speed_msg)
            msg = String()
            msg.data = 'C2'
            self.ros_node.InstRover.publish(msg)

    def Retroceder_rover(self, *args):
        if self.Modo.data == "M":
            self.ros_node.wheel_speed_msg.data = [self.vel_wheel, self.vel_wheel, self.vel_wheel,       # Velocidades de las ruedas del robot
                                                    -self.vel_wheel, -self.vel_wheel, -self.vel_wheel]
            self.ros_node.wheel_pos_msg.data = [0.0,0.0,0.0,0.0]               # Angulos de las ruedas del robot
            self.ros_node.dir_cmd_pos_pub_.publish(self.ros_node.wheel_pos_msg)
            sleep(1)
            self.ros_node.wheel_cmd_pub_.publish(self.ros_node.wheel_speed_msg)
            msg = String()
            msg.data = 'C1'
            self.ros_node.InstRover.publish(msg)
    
    def Izquierda_rover(self, *args):
        if self.Modo.data == "M":
            ICRl_ = 0.8 * self.vel_wheel*self.wheel_separation_/(self.vel_wheel-0.8 * self.vel_wheel)
            alpha_ = atan2(self.L1_,ICRl_+self.wheel_separation_/2)
            self.ros_node.wheel_speed_msg.data = [-0.8 * self.vel_wheel, -0.8 * self.vel_wheel, -0.8 *self.vel_wheel,       # Velocidades de las ruedas del robot
                                                    self.vel_wheel, self.vel_wheel, self.vel_wheel]
            angulo = alpha_ * 1331/(2*3.14159)
            self.ros_node.wheel_pos_msg.data = [angulo,-angulo,-angulo,angulo]  # Angulos de las ruedas del robot
            self.ros_node.dir_cmd_pos_pub_.publish(self.ros_node.wheel_pos_msg)
            sleep(1.5)
            self.ros_node.wheel_cmd_pub_.publish(self.ros_node.wheel_speed_msg)
            msg = String()
            msg.data = 'C1'
            self.ros_node.InstRover.publish(msg)
    
    def Derecha_rover(self, *args):
        if self.Modo.data == "M":
            ICRl_ = 0.8 * self.vel_wheel*self.wheel_separation_/(self.vel_wheel-0.8 * self.vel_wheel)
            alpha_ = atan2(self.L1_,ICRl_+self.wheel_separation_/2)
            self.ros_node.wheel_speed_msg.data = [-self.vel_wheel, -self.vel_wheel, -self.vel_wheel,       # Velocidades de las ruedas del robot
                                                    0.8 * self.vel_wheel, 0.8 * self.vel_wheel, 0.8 * self.vel_wheel]
            angulo = alpha_ * 1331/(2*3.14159)
            self.ros_node.wheel_pos_msg.data = [-angulo,angulo,angulo,-angulo]  # Angulos de las ruedas del robot
            self.ros_node.dir_cmd_pos_pub_.publish(self.ros_node.wheel_pos_msg)
            sleep(1.5)
            self.ros_node.wheel_cmd_pub_.publish(self.ros_node.wheel_speed_msg)
            msg = String()
            msg.data = 'C1'
            self.ros_node.InstRover.publish(msg)

    def GiroD_rover(self, *args):
        if self.Modo.data == "M":
            self.ros_node.wheel_speed_msg.data = [-self.vel_wheel, -self.vel_wheel, -self.vel_wheel,       # Velocidades de las ruedas del robot
                                                    -self.vel_wheel, -self.vel_wheel, -self.vel_wheel]
            angulo = 0.785398 * 1331/(2*3.14159)
            self.ros_node.wheel_pos_msg.data = [-angulo,angulo,-angulo,angulo]  # Angulos de las ruedas del robot
            self.ros_node.dir_cmd_pos_pub_.publish(self.ros_node.wheel_pos_msg)
            sleep(2)
            self.ros_node.wheel_cmd_pub_.publish(self.ros_node.wheel_speed_msg)
            msg = String()
            msg.data = 'C1'
            self.ros_node.InstRover.publish(msg)
    
    def GiroI_rover(self, *args):
        if self.Modo.data == "M":
            self.ros_node.wheel_speed_msg.data = [self.vel_wheel, self.vel_wheel, self.vel_wheel,       # Velocidades de las ruedas del robot
                                                    self.vel_wheel, self.vel_wheel, self.vel_wheel]
            angulo = 0.785398 * 1331/(2*3.14159)
            self.ros_node.wheel_pos_msg.data = [-angulo,angulo,-angulo,angulo]  # Angulos de las ruedas del robot
            self.ros_node.dir_cmd_pos_pub_.publish(self.ros_node.wheel_pos_msg)
            sleep(2)
            self.ros_node.wheel_cmd_pub_.publish(self.ros_node.wheel_speed_msg)
            msg = String()
            msg.data = 'C1'
            self.ros_node.InstRover.publish(msg)
    
    def Paro(self, *args):
        self.Modo.data = "M"
        self.ros_node.wheel_speed_msg.data = [0.0,0.0,0.0,       # Velocidades de las ruedas del robot
                                                0.0,0.0,0.0]
        self.ros_node.wheel_pos_msg.data = [0.0,0.0,0.0,0.0]  # Angulos de las ruedas del robot
        self.ros_node.dir_cmd_pos_pub_.publish(self.ros_node.wheel_pos_msg)
        self.ros_node.wheel_cmd_pub_.publish(self.ros_node.wheel_speed_msg)
        self.ros_node.InstRover.publish(self.Modo)
        msg = String()
        msg.data = 'C5'
        self.ros_node.InstRover.publish(msg)

    def ros_spin_once(self, dt):
        # Ejecuta una iteracion de rclpy.spin_once() para que ROS procese eventos
        rclpy.spin_once(self.ros_node, timeout_sec=0)

    def start_ros(self):
        # Inicializa ROS2 y el nodo
        rclpy.init()
        self.ros_node = ReceptorRover()
        # Ejecuta ros_spin_once cada 100ms
        Clock.schedule_interval(self.ros_spin_once, 0.1)

    def on_start(self):
        # Inicia ROS2 y programa la actualizacion periodica de la UI
        self.start_ros()
        # Clock.schedule_interval(self.update_message, 0.5)  # Actualiza la UI cada 0.5s

    def on_stop(self):
        # Al cerrar la app, detiene ROS2 correctamente
        if self.ros_node:
            rclpy.shutdown()

# Funcion principal
def main():
    ROS2KivyApp().run()

if __name__ == '__main__':
    main()