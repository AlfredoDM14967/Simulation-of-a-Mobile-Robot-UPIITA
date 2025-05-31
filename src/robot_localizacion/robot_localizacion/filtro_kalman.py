#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import String
import math
from rclpy.qos import qos_profile_sensor_data

class KalmanFilter(Node):
    def __init__(self):
        super().__init__("kalman_filter")

        self.odom_sub_ = self.create_subscription(Odometry, "ttrobot_controller/odom", self.odomCallback, 10) # Suscripcion a la odometria sin filtrar
        self.imu_sub_ = self.create_subscription(Imu, "imu/out", self.imuCallback, qos_profile=qos_profile_sensor_data)
        self.odom_pub_ = self.create_publisher(Odometry, "ttrobot_controller/odom_kalman", 10)  # Publicacion de la odometria filtrada
        self.gps_sub_ = self.create_subscription(NavSatFix, '/gps/fix',self.posCallback, 10)
        self.new_tra_sub_ = self.create_subscription(String, 'Comandos', self.Reinicio, 10)

        self.mean_ = 0.0
        self.variance_ = 1000.0

        self.imu_angular_z_ = 0.0       # Ultima medicion del IMU de su velocidad angular
        self.imu_orientation_z = 0.0       # Ultima medicion del IMU de su orientacion en el marco global
        self.is_first_odom_ = True      # Variable para identificar la primera lectura de la odometria /odom
        self.last_angular_z_ = 0.0      # Ultima estimacion de la velocidad angular proveniente de /odom
        self.is_first_gps = True

        self.motion_ = 0.0              # DIferencia de velocidad angular entre dos mediciones consecutivas
        self.kalman_odom_ = Odometry()
        self.kalman_odom_.header.frame_id = "Odom_kalman"
        self.kalman_odom_.child_frame_id = "base_footprint_ekf"
        self.motion_variance_ = 4.0

        self.measurement_variance_ = 0.5

        self.dLa = 0.0
        self.dLo = 0.0
        self.Laf = 0.0
        self.R = 6371000.0

        self.on_ = False

    def measurementUpdate(self):
        self.mean = (self.measurement_variance_ * self.mean_ + self.variance_ * self.imu_angular_z_)/(self.variance_ + self.measurement_variance_)
        self.variance_ = (self.variance_ * self.measurement_variance_) / (self.variance_ + self.measurement_variance_)

    def statePrediction(self):
        self.mean_ = self.mean_ + self.motion_
        self.variance_ = self.variance_ + self.motion_variance_
    
    def imuCallback(self, imu):
        a = 1.0
        # self.imu_angular_z_ = imu.angular_velocity.z
        # self.imu_orientation_z = imu.orientation._z  # Lectura del magnetometro de la IMU orientacion sobre el eje z

    def odomCallback(self, odom):
        if self.on_:
            self.kalman_odom_ = odom
            
            self.kalman_odom_.header.frame_id = "Odom_kalman"
            self.kalman_odom_.child_frame_id = "base_footprint_ekf"
            
            if self.is_first_odom_:
                self.mean_ = odom.twist.twist.angular.z
                self.last_angular_z_ = odom.twist.twist.angular.z

                #   Establecimiento de la orientacion inicial
                self.kalman_odom_.pose.pose.orientation.z = self.imu_orientation_z

                self.is_first_odom_ = False
                return
            
            self.motion_ = odom.twist.twist.angular.z - self.last_angular_z_
            
            self.statePrediction()

            self.measurementUpdate()

            self.kalman_odom_.twist.twist.angular.z = self.mean_
            self.kalman_odom_.pose.pose.orientation.z = self.imu_orientation_z#odom.pose.pose.orientation.z

            X = self.R * 0.0174533 * self.dLo * math.cos(0.0174533 * self.Laf)  # Longitud
            Y = self.R * 0.0174533 * self.dLa                                   # Latitud

            self.kalman_odom_.pose.pose.position.x = X   # Se agrega el desplazamiento de la odometria en X
            self.kalman_odom_.pose.pose.position.y = Y   # y en Y

            self.odom_pub_.publish(self.kalman_odom_)

            # self.get_logger().info("X: %f Y: %f" % (X,Y))
    
    def posCallback(self, gps):
        if self.on_:
            if self.is_first_gps :
                self.Lai = gps.latitude
                self.Loi = gps.longitude

                self.is_first_gps = False

            self.Laf = gps.latitude
            Lof = gps.longitude

            self.dLa = self.Laf - self.Lai
            self.dLo = Lof - self.Loi
    
    def Reinicio(self, msg):
        if msg.data == 'A':
            self.on_ = True
            self.is_first_gps = True
            self.is_first_odom_ = True
            self.motion_variance_ = 4.0
            self.measurement_variance_ = 0.5
            self.motion_ = 0.0
            self.last_angular_z_ = 0.0 
            self.mean_ = 0.0
            self.variance_ = 1000.0
            

def main():
    rclpy.init()
    kalman_filter = KalmanFilter()
    rclpy.spin(kalman_filter)
    kalman_filter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()