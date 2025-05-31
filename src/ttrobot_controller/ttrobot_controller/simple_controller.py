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
from enum import Enum
from time import sleep

class RobotState(Enum):
    ROTAR = 1
    AVANZAR = 2
    DETERMINAR_GIRO = 3
    GIRO_DERECHA = 4
    GIRO_IZQUIERDA = 5
    SUPERAR_OBSTACULO = 6

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
        self.Comand_pub_ = self.create_publisher(String, 'Comandos', 10)
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

        # #Constantes PI
        self.Kpx = 1.3#0.8
        self.Kpy = 1.3#0.8
        self.Kpth = 1.3

        self.error1_sum = 0
        self.error2_sum = 0
        self.u1 = 0
        self.u2 = 0

        #Posicion deseada
        self.xd = 0.0
        self.yd = 0.0
        self.is_first_gps_ = False

        # Inicializacion de variables
        self.state = RobotState.ROTAR
        self.e_w = 0.0  # Error de orientacion
        self.e_p = 0.0  # Error de posicion
        self.F_ = False  # Objeto frontal
        self.D_ = False  # Objeto lateral derecho
        self.I_ = False  # Objeto lateral izquierdo
        self.Gd = False  # Giro a la derecha
        self.Gi = False  # Giro a la izquierda
        self.rotation_accumulated = 0.0  # Rotacion acumulada (en radianes)
        self.is_first_rotation = True
        self.giro_completed = False
        self.mdir_ang_ = [0.0, 0.0, 0.0, 0.0]

    def velCallback(self, msg):

        robot_speed = np.array([[msg.twist.linear.x],       # Velocidad lineal m/s y angular rad/s
                                [msg.twist.angular.z]])

        wheel_speed = np.matmul(np.linalg.inv(self.speed_conversion_), robot_speed)     # Obtencion de la velocidad de las ruedas
        wheel_speed_msg = Float64MultiArray()
        if abs(msg.twist.angular.z) > abs(msg.twist.linear.x):
            if wheel_speed[0,0] > 1.5:
                wheel_speed[0,0] = 0.8
            elif wheel_speed[0,0] < -1.5:
                wheel_speed[0,0] = -0.8
            if wheel_speed[1,0] > 1.5:
                wheel_speed[1,0] = 0.8
            elif wheel_speed[1,0] < -1.5:
                wheel_speed[1,0] = -0.8
        else:
            if wheel_speed[0,0] > 2.5:
                wheel_speed[0,0] = 2.5
            elif wheel_speed[0,0] < -2.5:
                wheel_speed[0,0] = -2.5
            if wheel_speed[1,0] > 2.5:
                wheel_speed[1,0] = 2.5
            elif wheel_speed[1,0] < -2.5:
                wheel_speed[1,0] = -2.5
        wheel_speed_msg.data = [-wheel_speed[0, 0], -wheel_speed[0, 0], -wheel_speed[0, 0],       # Velocidades de las ruedas del robot rad/s
                                wheel_speed[1, 0],wheel_speed[1, 0], wheel_speed[1, 0]]

        angulo_direc_msg = Float64MultiArray()
        if abs(msg.twist.angular.z) > abs(msg.twist.linear.x) :
            angulo = 0.785398 * 1331/(2*3.14159)
            angulo_direc_msg.data = [-angulo,angulo,-angulo,angulo]   # Angulo de 45° si se requiere rotar sobre el eje z del robot
        else:
            angulo = 0.0
            angulo_direc_msg.data = [-angulo,angulo,-angulo,angulo]

        self.dir_cmd_pos_pub_.publish(angulo_direc_msg)
        ang1 = angulo_direc_msg.data[0] - self.mdir_ang_[0]
        ang2 = angulo_direc_msg.data[0] - self.mdir_ang_[1]
        ang3 = angulo_direc_msg.data[0] - self.mdir_ang_[2]
        ang4 = angulo_direc_msg.data[0] - self.mdir_ang_[3]
        prom = (ang1 + ang2 + ang3 + ang4) / 4.0
        self.get_logger().info("Error Direccion: %f" % abs(ang1))
        if (abs(ang1) <= 40.0): # and (abs(ang2) <= 0.7) (abs(ang3) <= 0.7) and (abs(ang4) <= 0.7) :
            self.wheel_cmd_pub_.publish(wheel_speed_msg)
        else:
            wheel_speed_msg.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            self.wheel_cmd_pub_.publish(wheel_speed_msg)

    def jointCallback(self, msg):
        wheel_encoder_left = (msg.position[0] + msg.position[1] + msg.position[2]) / 3 # Promedio de las lecturas del encoder
        wheel_encoder_right = (msg.position[3] + msg.position[4] + msg.position[5]) / 3
        for i in range (0,4):
            self.mdir_ang_[i] = msg.position[i+6]
        # self.get_logger().info("WR: %f WL: %f" % (wheel_encoder_right, wheel_encoder_left))
        dp_left = wheel_encoder_left - self.left_wheel_prev_pos_
        dp_right = wheel_encoder_right - self.right_wheel_prev_pos_
        # dt = Time.from_msg(msg.header.stamp) - self.prev_time_

        self.left_wheel_prev_pos_ = (msg.position[0] + msg.position[1] + msg.position[2]) / 3  # Actualizacion de valores previos de la 
        self.right_wheel_prev_pos_ = (msg.position[3] + msg.position[4] + msg.position[5]) / 3 # lectura de encoders y tiempo de lectura de datos
        self.prev_time_ = Time.from_msg(msg.header.stamp)

        fi_left = (msg.velocity[0] + msg.velocity[1] + msg.velocity[2]) / 3  #dp_left / (dt.nanoseconds / S_TO_NS)      # Velocidad angular rueda izquierda y derecha
        fi_right = (msg.velocity[3] + msg.velocity[4] + msg.velocity[5]) / 3 #3dp_right / (dt.nanoseconds / S_TO_NS)

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

        self.odom_pub_.publish(self.odom_msg_)

    def odom_filtered(self, msg):
        if self.on_:
            self.error1_sum = self.xd - msg.pose.pose.position.x    # Error en X longitud
            self.error2_sum = self.yd - msg.pose.pose.position.y    # Error en Y latitud
            R_error = math.sqrt(math.pow(self.error1_sum, 2) + math.pow(self.error2_sum, 2))
            self.e_w = math.atan2(self.error1_sum, self.error2_sum)
            if self.e_w < 0:
                self.e_w += 6.28318530718
            self.e_w -= msg.pose.pose.orientation.z
            self.get_logger().info("El error X: %f Y: %f W: %f" % (self.error1_sum, self.error2_sum, self.e_w))

            if self.state == RobotState.ROTAR:
                self.get_logger().info("Estado: ROTAR")
                if (self.e_w > 0.22) or (self.e_w < -0.22):
                    self.rotate()
                else:
                    self.state = RobotState.AVANZAR
            
            elif self.state == RobotState.AVANZAR:
                self.get_logger().info("Estado: AVANZAR")
                if (self.e_w > 0.22) or (self.e_w < -0.22):
                    self.state = RobotState.ROTAR
                if R_error >= 1.4:
                    self.move_forward(msg.pose.pose.orientation.z)
                    if self.F_:
                        self.state = RobotState.DETERMINAR_GIRO
                else:
                    self.get_logger().info("Objetivo alcanzado")
                    velocidades = TwistStamped()
                    velocidades.twist.linear.x = 0.0
                    velocidades.twist.angular.z = 0.0
                    self.on_ = False

                    self.velocity_pub_.publish(velocidades)

                    msg = String()
                    msg.data = "F"
                    self.Comand_pub_.publish(msg)

            elif self.state == RobotState.DETERMINAR_GIRO:
                self.get_logger().info("Estado: DETERMINAR_GIRO")
                velocidades = TwistStamped()
                velocidades.twist.linear.x = 0.0
                velocidades.twist.angular.z = 0.0

                self.velocity_pub_.publish(velocidades)
                if not self.D_:
                    self.get_logger().info("Giro D")
                    self.state = RobotState.GIRO_DERECHA
                    self.Gd = True
                    velocidades = TwistStamped()
                    velocidades.twist.linear.x = 0.0
                    velocidades.twist.angular.z = 0.0

                    self.velocity_pub_.publish(velocidades)
                elif not self.I_:
                    self.get_logger().info("Giro I")
                    self.state = RobotState.GIRO_IZQUIERDA
                    self.Gi = True
            
            elif self.state == RobotState.GIRO_DERECHA:
                self.get_logger().info("Estado: GIRO_DERECHA")
                if self.is_first_rotation:
                    self.rotation_accumulated = (msg.pose.pose.orientation.z + 1.5708) % 6.28319
                    self.is_first_rotation = False
                self.rotate_to_angle(msg.pose.pose.orientation.z)  # Rotar 90º a la derecha
                if self.giro_completed:
                    self.state = RobotState.SUPERAR_OBSTACULO
            
            elif self.state == RobotState.GIRO_IZQUIERDA:
                self.get_logger().info("Estado: GIRO_IZQUIERDA")
                if self.is_first_rotation:
                    self.rotation_accumulated = (msg.pose.pose.orientation.z - 1.5708) % 6.28319
                    self.is_first_rotation = False
                self.rotate_to_angle(msg.pose.pose.orientation.z)  # Rotar 90º a la izquierda
                if self.giro_completed:
                    self.state = RobotState.SUPERAR_OBSTACULO

            elif self.state == RobotState.SUPERAR_OBSTACULO:
                self.get_logger().info("Estado: SUPERAR_OBSTACULO")
                self.avoid_obstacle()
                if (self.Gd and not self.I_) or (self.Gi and not self.D_):
                    self.Gd = False
                    self.Gi = False
                    self.giro_completed = False
                    self.is_first_rotation = True
                    self.state = RobotState.ROTAR


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
            # self.get_logger().info("xd: %f yd: %f"  % (self.xd, self.yd))

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
        
    def rotate(self):
        velocidades = TwistStamped()
        w =self.Kpth * -(self.e_w)
        velocidades.twist.linear.x = 0.0
        velocidades.twist.angular.z = w  
        self.velocity_pub_.publish(velocidades)

    def move_forward(self, orientation):
        velocidades = TwistStamped()
        self.u1 = (self.Kpx * self.error1_sum * math.sin(orientation) + self.Kpy * self.error2_sum * math.cos(orientation))
        self.u2 = ((-1.0 / self.L1_) * self.Kpx * self.error1_sum * math.cos(orientation)
                + (1.0 / self.L1_) * self.Kpy * self.error2_sum * math.sin(orientation))

        velocidades.twist.linear.x = abs(self.u1)
        velocidades.twist.angular.z = 0.0
        # self.get_logger().info("u1: %f u2: %f" % (self.u1, self.u2))
        self.velocity_pub_.publish(velocidades)
    
    def rotate_to_angle(self, ang):
        velocidades = TwistStamped()
        th_error = self.rotation_accumulated - ang
        w = -self.Kpth * th_error 
        velocidades.twist.linear.x = 0.0
        velocidades.twist.angular.z = w  
        self.velocity_pub_.publish(velocidades)
        self.get_logger().info("Error Giro: %f" % th_error)
        if abs(th_error) <= 0.2:
            self.giro_completed = True
    
    def avoid_obstacle(self):
        velocidades = TwistStamped()
        velocidades.twist.linear.x = 3.0
        velocidades.twist.angular.z = 0.0 
        self.velocity_pub_.publish(velocidades)
            
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