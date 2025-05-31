# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import String


# class rover_IHMremota(Node):

#     def __init__(self):
#         super().__init__("Rover_IHMremota")
#         # Nodos publicacion de comunicación del usario
#         self.pub_Vid = self.create_publisher(String, "Video", 10)
#         self.pub_Dat = self.create_publisher(String, "Datos_rover", 10)
#         self.counter_ = 0
#         self.frequency_ = 5.0
#         self.get_logger().info("Publishing at %d Hz" % self.frequency_)

#         # Nodos subscripcion de comunicación del usuario
#         self.sub_MOp = self.create_subscription(String, "Modo_Operacion", self.msgCallback, 10)
#         self.sub_Inst = self.create_subscription(String, "Instrucciones", self.InstCallback, 10)

#         # Nodos subscripcion de comunicación del Resource Manager
#         self.sub_Cam = self.create_subscription(String, "Image", self.CamCallback, 10)
#         self.sub_CM = self.create_subscription(String, "Controller_manager", self.CMCallback, 10)
#         self.sub_GPS = self.create_subscription(String, "NavStatFix", self.GPSCallback, 10)
#         self.sub_Mg = self.create_subscription(String, "MagneticField", self.MgCallback, 10)
#         self.sub_LD = self.create_subscription(String, "scan", self.LDCallback, 10)
#         self.sub_Mot = self.create_subscription(String, "Hub", self.MotCallback, 10)
#         self.sub_STM = self.create_subscription(String, "STM", self.STMCallback, 10)

#         self.timer_ = self.create_timer(self.frequency_, self.timerCallback)
#         self.timer_ = self.create_timer(self.frequency_, self.VidCallback)

#     def timerCallback(self):
#         msg = String()
#         msg.data = "Video: %d" % self.counter_
#         self.pub_Vid.publish(msg)
#         self.counter_ += 1

#     def VidCallback(self):
#         msg = String()
#         msg.data = "Datos del rover: %d" % self.counter_
#         self.pub_Dat.publish(msg)

#     def msgCallback(self, msg):
#         self.get_logger().info("Modo de operacion: %s" % msg.data)

#     def InstCallback(self, msg):
#         self.get_logger().info("Instrucciones: %s" % msg.data)

#     def CamCallback(self, msg):
#         self.get_logger().info("Camara: %s" % msg.data)

#     def CMCallback(self, msg):
#         self.get_logger().info("Control Manager: %s" % msg.data)

#     def GPSCallback(self, msg):
#         self.get_logger().info("GPS: %s" % msg.data)

#     def MgCallback(self, msg):
#         self.get_logger().info("Magnetometro: %s" % msg.data)

#     def LDCallback(self, msg):
#         self.get_logger().info("LIDAR: %s" % msg.data)

#     def MotCallback(self, msg):
#         self.get_logger().info("HUB: %s" % msg.data)

#     def STMCallback(self, msg):
#         self.get_logger().info("STM: %s" % msg.data)


# def main():
#     rclpy.init()

#     simple_publisher = rover_IHMremota()
#     rclpy.spin(simple_publisher)
#     simple_publisher.destroy_node()
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()


from kivy.app import App
from kivy.uix.image import Image
from kivy.graphics.texture import Texture
from kivy.clock import Clock

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image as RosImage
from cv_bridge import CvBridge
import threading

class VideoSubscriber(Node):
    def __init__(self, callback):
        super().__init__('video_subscriber')
        self.subscription = self.create_subscription(RosImage, 'video_frames', self.listener_callback, 10)
        self.bridge = CvBridge()
        self.callback = callback

    def listener_callback(self, data):
        # Convertir imagen de ROS2 a OpenCV
        frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
        self.callback(frame)

class KivyVideoApp(App):
    def build(self):
        # Crear una imagen vacía en la interfaz
        self.img_widget = Image()
        Clock.schedule_once(self.start_ros, 0)
        return self.img_widget

    def start_ros(self):
        # Iniciar ROS2 en un hilo aparte
        def ros_thread():
            rclpy.init()
            self.ros_node = VideoSubscriber(self.update_frame)
            rclpy.spin(self.ros_node)
            rclpy.shutdown()
        threading.Thread(target=ros_thread).start()

    def update_frame(self, frame):
        # Convertir frame OpenCV a textura Kivy
        buffer = cv2.flip(frame, 0).tostring()  # Kivy usa coordenadas invertidas
        texture = Texture.create(size=(frame.shape[1], frame.shape[0]), colorfmt='bgr')
        texture.blit_buffer(buffer, colorfmt='bgr', bufferfmt='ubyte')
        self.img_widget.texture = texture

    def on_stop(self):
        self.ros_node.destroy_node()

if __name__ == '__main__':
    KivyVideoApp().run()


# from kivymd.app import MDApp
# from kivymd.uix.label import MDLabel


# class MainApp(MDApp):
#     def build(self):
#         return MDLabel(text="Hello, World", halign="center")


# MainApp().run()