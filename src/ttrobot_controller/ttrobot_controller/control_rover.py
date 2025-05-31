#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String
from geometry_msgs.msg import TransformStamped, TwistStamped
from sensor_msgs.msg import JointState
import numpy as np
from rclpy.time import Time
from rclpy.constants import S_TO_NS
from nav_msgs.msg import Odometry
import math
from transforms3d.euler import euler2quat
import tf_transformations
from tf2_ros import TransformBroadcaster
from sensor_msgs.msg import NavSatFix

class NoisyController(Node):
    def __init__(self):
        super().__init__("simple_controller")

        self.declare_parameter("wheel_radius",0.105)
        self.declare_parameter("wheel_separation",1.115)
        self.declare_parameter("L1",0.70)

        self.wheel_radius_ = self.get_parameter("wheel_radius").get_parameter_value().double_value
        self.wheel_separation_ = self.get_parameter("wheel_separation").get_parameter_value().double_value
        self.L1_ = self.get_parameter("L1").get_parameter_value().double_value

        self.get_logger().info("Usando wheel_radius %r" % self.wheel_radius_)
        self.get_logger().info("Usando wheel_separation %r" % self.wheel_separation_)
        self.get_logger().info("Usando L1 %r" % self.L1_)
        
        self.left_wheel_prev_pos_ = 0.0
        self.right_wheel_prev_pos_ = 0.0
        self.prev_time_ = self.get_clock().now()

        self.x_ = 0.0
        self.y_ = 0.0
        self.theta_ = 0.0

        self.joint_sub_ = self.create_subscription(JointState, "joint_states", self.jointCallback, 10)
        self.odom_pub_ = self.create_publisher(Odometry, "ttrobot_controller/odom", 10)

        #publicador y subscriptor de velocidad
        self.wheel_cmd_pub_ = self.create_publisher(Float64MultiArray, "simple_velocity_controller/commands", 10)
        self.vel_sub_ = self.create_subscription(TwistStamped, "ttrobot_controller/cmd_vel", self.velCallback, 10)

        #publicador y subscriptor de posicion
        self.dir_cmd_pos_pub_ = self.create_publisher(Float64MultiArray, "simple_position_controller/commands", 10)
        # self.pos_sub_ = self.create_subscription(TwistStamped, "ttrobot_controller/cmd_pos", self.velCallback, 10)

        #Suscripcion a la odometria filtrada
        self.odom_sub_kalman_ = self.create_subscription(Odometry, "ttrobot_controller/odom_kalman", self.odom_filtered, 10)

        #Suscripcion al gps para establecer punto final deseado
        self.gps_sub_ = self.create_subscription(NavSatFix, '/gps/fix',self.posfCallback, 10)

        #Suscripcion al topico de la posicion deseada
        self.posdeseada_sub_ = self.create_subscription(String, '/Position',self.posdCallback, 10)

        #Publicador de velocidad lineal y angular
        self.velocity_pub_ = self.create_publisher(TwistStamped, "ttrobot_controller/cmd_vel", 10)

        #Subscriptor deteccion de obstaculos LIDAR
        self.obst_sub = self.create_subscription(String, "/LIDAR", self.obstCallback, 10)

        #Subscribir para iniciar navegacion
        self.new_tra_sub_ = self.create_subscription(String, 'Comandos', self.Reinicio, 10)
        self.on_ = False

        #Matriz de conversion
        self.speed_conversion_ = np.array([[self.wheel_radius_/2, self.wheel_radius_/2],
                                          [-self.wheel_radius_/self.wheel_separation_, self.wheel_radius_/self.wheel_separation_]])
        self.get_logger().info("La matriz de conversion es %s" % self.speed_conversion_)
 
        self.odom_msg_ = Odometry()
        self.odom_msg_.header.frame_id = "odom"
        self.odom_msg_.child_frame_id = "base_footprint_ekf"
        self.odom_msg_.pose.pose.orientation.x = 0.0
        self.odom_msg_.pose.pose.orientation.y = 0.0
        self.odom_msg_.pose.pose.orientation.z = 0.0
        self.odom_msg_.pose.pose.orientation.w = 1.0

        # Mensaje de odometria filtrada
        # self.odom_msg_k_ = Odometry()
        # self.odom_msg_k_.header.frame_id = "base_link"
        # self.odom_msg_k_.child_frame_id = "base_footprint"
        # self.odom_msg_k_.pose.pose.orientation.x = 0.0
        # self.odom_msg_k_.pose.pose.orientation.y = 0.0
        # self.odom_msg_k_.pose.pose.orientation.z = 0.0
        # self.odom_msg_k_.pose.pose.orientation.w = 1.0

        self.br_ = TransformBroadcaster(self)
        self.transform_stamped_ = TransformStamped()
        self.transform_stamped_.header.frame_id = "odom"
        self.transform_stamped_.child_frame_id = "base_footprint_noisy"

        # #Constantes PI
        self.Kpx = 1.3#0.8
        self.Kpy = 1.3#0.8

        self.error1_sum = 0
        self.error2_sum = 0
        self.u1 = 0
        self.u2 = 0

        #Posicion deseada
        self.xd = 0.0
        self.yd = 0.0
        self.is_first_gps_ = False

        #Variables de deteccion de obstaculos F: Obstaculo frontal, D: Obstaculo lateral derecho, I: Obstaculo lateral izquierdo
        self.F_ = False
        self.D_ = False
        self.I_ = False

        self.Avance_ = True
        self.Orientacion_ = [True,True,True,True,0.0] # [0] Bandera de tomar orientacion actual para rotar 90DEG [1] Bandera de ejecutar giro Izq o Der
        # [2] Bandera de giro izquierda [3] Bandera giro derecha [4] Registro para guardar orientacion al momento de detectar obstaculo 

    def velCallback(self, msg):

        robot_speed = np.array([[msg.twist.linear.x],       # Velocidad lineal m/s y angular rad/s
                                [msg.twist.angular.z]])

        wheel_speed = np.matmul(np.linalg.inv(self.speed_conversion_), robot_speed)     # Obtencion de la velocidad de las ruedas
        wheel_speed_msg = Float64MultiArray()
        if wheel_speed[0,0] > 3.0:
            wheel_speed[0,0] = 1.7
        elif wheel_speed[0,0] < -3.0:
            wheel_speed[0,0] = -1.7
        if wheel_speed[1,0] > 3.0:
            wheel_speed[1,0] = 1.7
        elif wheel_speed[1,0] < -3.0:
            wheel_speed[1,0] = -1.7
        wheel_speed_msg.data = [-wheel_speed[0, 0], -wheel_speed[0, 0], -wheel_speed[0, 0],       # Velocidades de las ruedas del robot rad/s
                                wheel_speed[1, 0],wheel_speed[1, 0], wheel_speed[1, 0]]

        #Calculo del angulo de la direccion
        # if (wheel_speed[1,0]-wheel_speed[0,0])<=0.03:
        #     alpha_ = 0.0
        # else :
        #     ICRl_ = wheel_speed[0,0]*self.wheel_separation_/(wheel_speed[1,0]-wheel_speed[0,0]) # Calculo del centro instantaneo de rotacion
        #     alpha_ = math.atan2(self.L1_,ICRl_+self.wheel_separation_/2)                        # Angulo de las ruedas de direccion
        # self.get_logger().info("WL: %f  WR: %f a: %f" %(wheel_speed[0,0],wheel_speed[1,0],alpha_))

        angulo_direc_msg = Float64MultiArray()
        if abs(msg.twist.angular.z) > 2.0 * abs(msg.twist.linear.x) :
            angulo = 0.785398 * 1331/(2*3.14159)
            angulo_direc_msg.data = [-angulo,angulo,-angulo,angulo]   # Angulo de 45° si se requiere rotar sobre el eje z del robot
        else:
            angulo = 0.0 * 1331/(2*3.14159)
            angulo_direc_msg.data = [-angulo,angulo,-angulo,angulo]

        self.dir_cmd_pos_pub_.publish(angulo_direc_msg)
        self.wheel_cmd_pub_.publish(wheel_speed_msg)

    def jointCallback(self, msg):
        wheel_encoder_left = (msg.position[0] + msg.position[1] + msg.position[2]) / 3 # Promedio de las lecturas del encoder
        wheel_encoder_right = (msg.position[3] + msg.position[4] + msg.position[5]) / 3
        # self.get_logger().info("WR: %f WL: %f" % (wheel_encoder_right, wheel_encoder_left))
        dp_left = wheel_encoder_left - self.left_wheel_prev_pos_
        dp_right = wheel_encoder_right - self.right_wheel_prev_pos_
        dt = Time.from_msg(msg.header.stamp) - self.prev_time_

        self.left_wheel_prev_pos_ = (msg.position[0] + msg.position[1] + msg.position[2]) / 3  # Actualizacion de valores previos de la 
        self.right_wheel_prev_pos_ = (msg.position[3] + msg.position[4] + msg.position[5]) / 3 # lectura de encoders y tiempo de lectura de datos
        self.prev_time_ = Time.from_msg(msg.header.stamp)

        fi_left = dp_left / (dt.nanoseconds / S_TO_NS)      # Velocidad angular rueda izquierda y derecha
        fi_right = dp_right / (dt.nanoseconds / S_TO_NS)

        linear =  (self.wheel_radius_ * fi_right + self.wheel_radius_ * fi_left) / 2                       # Velocidad lineal y angular del robot
        angular = (self.wheel_radius_ * fi_right - self.wheel_radius_ * fi_left) / self.wheel_separation_

        d_s = (self.wheel_radius_ * dp_right + self.wheel_radius_ * dp_left) / 2                            # Variacion en la posicion
        d_theta = (self.wheel_radius_ * dp_right - self.wheel_radius_ * dp_left) / self.wheel_separation_   # Variacion en la orientacion
        self.theta_ += d_theta                      # Orientacion actual
        self.x_ += d_s * math.cos(self.theta_)      # Posicion actual "x" y "y"
        self.y_ += d_s * math.sin(self.theta_)

        q = tf_transformations.quaternion_from_euler(0, 0, self.theta_)
        self.odom_msg_.pose.pose.orientation.x = q[0]
        self.odom_msg_.pose.pose.orientation.y = q[1]
        self.odom_msg_.pose.pose.orientation.z = q[2]
        self.odom_msg_.pose.pose.orientation.w = q[3]
        self.odom_msg_.header.stamp = self.get_clock().now().to_msg()
        self.odom_msg_.pose.pose.position.x = self.x_
        self.odom_msg_.pose.pose.position.y = self.y_
        self.odom_msg_.twist.twist.linear.x = linear
        self.odom_msg_.twist.twist.angular.z = angular

        self.transform_stamped_.transform.translation.x = self.x_
        self.transform_stamped_.transform.translation.y = self.y_
        self.transform_stamped_.transform.rotation.x = q[0]
        self.transform_stamped_.transform.rotation.y = q[1]
        self.transform_stamped_.transform.rotation.z = q[2]
        self.transform_stamped_.transform.rotation.w = q[3]
        self.transform_stamped_.header.stamp = self.get_clock().now().to_msg()

        # self.get_logger().info("linear: %f, angular: %f" % (linear, angular))
        #self.get_logger().info("x: %f, y: %f, theta: %f" % (self.x_, self.y_, self.theta_))

        self.odom_pub_.publish(self.odom_msg_)
        # self.br_.sendTransform(self.transform_stamped_)   # Publica la posicion y orientacion del robot para ver en RVIZ2

    def odom_filtered(self, msg):
        if self.on_:
            velocidades = TwistStamped()
            self.error1_sum = self.xd - msg.pose.pose.position.x    # Error en X longitud
            self.error2_sum = self.yd - msg.pose.pose.position.y    # Error en Y latitud
            R_error = math.sqrt(math.pow(self.error1_sum, 2) + math.pow(self.error2_sum, 2))
            self.get_logger().info("El error X: %f Y: %f" % (self.error1_sum, self.error2_sum))
            if not(self.F_) and self.Avance_ and (R_error >= 2.0):
                
                # quat = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w]
                # x,y,z = tf_transformations.euler_from_quaternion(quat)
                self.u1 = (self.Kpx * self.error1_sum * math.sin(msg.pose.pose.orientation.z) + self.Kpy * self.error2_sum * math.cos(msg.pose.pose.orientation.z))
                self.u2 = ((-1.0 / self.L1_) * self.Kpx * self.error1_sum * math.cos(msg.pose.pose.orientation.z)
                        + (1.0 / self.L1_) * self.Kpy * self.error2_sum * math.sin(msg.pose.pose.orientation.z))

                velocidades.twist.linear.x = self.u1
                velocidades.twist.angular.z = self.u2
                self.get_logger().info("u1: %f u2: %f" % (self.u1, self.u2))

            elif not(self.Avance_) and (R_error >= 2.0):
                if (self.D_ or self.Orientacion_[2]) and self.Orientacion_[1]:
                    self.Giro("I")
                    self.Orientacion_[2] = True
                elif (self.I_ or self.Orientacion_[3]) and self.Orientacion_[1]:
                    self.Giro("D")
                    self.Orientacion_[3] = True
                if self.Orientacion_[0]:   
                    velocidades.twist.linear.x = 0.0
                    velocidades.twist.angular.z = 0.0
                    self.Orientacion_[4] = msg.pose.pose.orientation.z
                    self.Orientacion_[0] = False
                if abs(self.Orientacion_[4] - msg.pose.pose.orientation.z) > 1.57079632679:
                    self.Orientacion_[1] = False
                    velocidades.twist.linear.x = 0.0
                    velocidades.twist.angular.z = 0.0
                elif (self.Orientacion_[2] or self.Orientacion_[3]) and not(self.Orientacion_[1]):
                    if self.Orientacion_[2] and self.D_:
                        velocidades.twist.linear.x = 1.4
                        velocidades.twist.angular.z = 0.0
                    elif self.Orientacion_[3] and self.I_:
                        velocidades.twist.linear.x = 1.4
                        velocidades.twist.angular.z = 0.0
                    else:
                        self.Orientacion_ = [True,True,True,True,0.0] # Se considera obstaculo superado y se reinician variables
            
            if R_error < 2.0:
                velocidades.twist.linear.x = 0.0
                velocidades.twist.angular.z = 0.0

            self.velocity_pub_.publish(velocidades)


    def posfCallback(self, pos):
        if self.is_first_gps_:
            Lai = pos.latitude
            Loi = pos.longitude

            Laf = self.yd
            Lof = self.xd

            dLa = Laf - Lai
            dLo = Lof - Loi

            R = 6371000

            self.xd = R * 0.0174533 * dLo * math.cos(0.0174533 * Laf)           # Longitud
            self.yd = R * 0.0174533 * dLa                                      # Latitud

            self.is_first_gps_ = False
            self.get_logger().info("xd: %f yd: %f"  % (self.xd, self.yd))

            # self.destroy_subscription(self.gps_sub_)

    def posdCallback(self, msg):        # Funcion para establecer posicion deseada
        POS = str(msg.data).split()
        self.xd = float(POS[0])
        self.yd = float(POS[1])
        self.is_first_gps_ = True
        # self.destroy_subscription(self.posdeseada_sub_)

    def obstCallback(self, msg):
        Obstaculo = str(msg.data).split()
        if Obstaculo[0] == "1" :
            self.F_ = True          # Obstaculo detectado
        else :
            self.F_ = False         # Sin obstaculo
        if Obstaculo[1] == "1" :
            self.D_ = True
        else :
            self.D_ = False
        if Obstaculo[2] == "1" :
            self.I_ = True
        else :
            self.I_ = False
        
    def Giro(self,Giro):
        velocidades = TwistStamped()
        if Giro == "I":
            velocidades.twist.linear.x = 0.0
            velocidades.twist.angular.z = -1.0
        elif Giro == "D":
            velocidades.twist.linear.x = 0.0
            velocidades.twist.angular.z = 1.0
            
    def Reinicio(self, msg):
        if msg.data == 'A':
            self.on_ = True 
        elif msg.data == 'M' or msg.data == 'D':
            self.on_ = False

def main():
    rclpy.init()
    noisy_controller = NoisyController()
    rclpy.spin(noisy_controller)
    noisy_controller.destroy_node()
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()