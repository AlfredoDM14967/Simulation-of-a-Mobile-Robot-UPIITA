#!/usr/bin/env python3
import rclpy.time
import smbus
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, MagneticField
from math import atan2
import numpy
from tf_transformations import quaternion_from_euler
from time import sleep

# Direcciones de registro para el IMU01A
ACCEL_XOUT_H  = 0x28  # Registro de salida de acelerometro (bajo)
ACCEL_YOUT_H  = 0x2A  # Registro de salida de acelerometro (bajo)
ACCEL_ZOUT_H  = 0x2C  # Registro de salida de acelerometro (bajo)
GYRO_XOUT_H   = 0x28  # Registro de salida de giroscopio (bajo)
GYRO_YOUT_H   = 0x2A  # Registro de salida de giroscopio (bajo)
GYRO_ZOUT_H   = 0x2C  # Registro de salida de giroscopio (bajo)
MAG_XOUT_H = 0x03     # Registro de salida de magnetometro (alto)
MAG_YOUT_H = 0x05     # Registro de salida de magnetometro (alto)
MAG_ZOUT_H = 0x07     # Registro de salida de magnetometro (alto)
CTRL_REG1     = 0x20  # Registro de control para configurar el sensor
ACCEL_ADDRESS = 0x18  # Direccion I2C del IMU01A (0x69 Giroscopio, 18 Acelerometro, 1E Magnetometro)
GYRO_ADDRESS = 0x69
MAGN_ADDRESS = 0x1E
CRA_REG_M = 0x00      # Configuracion del magnetometro
CRB_REG_M = 0x01
MR_REG = 0x02         # Modo de operacion del magnetometro

MAGN_ADDRESS_2 = 0x1E   #Direccion magnetometro del GPS HMC5883L
CONF_A = 0x00
CONF_B = 0x01
MODE_REG = 0x02
MAG_XOUT_H_2 = 0x03     # Registro de salida de magnetometro del GPS (alto)
MAG_YOUT_H_2 = 0x05     # Registro de salida de magnetometro del GPS (alto)
MAG_ZOUT_H_2 = 0x07     # Registro de salida de magnetometro del GPS (alto)

class IMU01A_Driver(Node):
    def __init__(self):
        super().__init__("imu01a_driver")
        
        # Interfaz I2C
        self.is_connected_ = False
        self.init_i2c()

        # Interfaz ROS 2
        self.imu_pub_ = self.create_publisher(Imu, "/imu/out", qos_profile=qos_profile_sensor_data)
        self.mag_pub_ = self.create_publisher(MagneticField, "/mag/out", qos_profile=qos_profile_sensor_data)
        self.imu_msg_ = Imu()
        self.imu_msg_.header.frame_id = "base_footprint"
        self.frequency_ = 0.01
        self.timer_ = self.create_timer(self.frequency_, self.timerCallback)
        self.Xsf =  0.5515 #0.5719
        self.Ysf =  5.35 #3.9760
        self.Xoff = -0.2110 #-0.0376
        self.Yoff = -0.2939 #-0.1968

        self.Mag = MagneticField()

    def timerCallback(self):
        try:
            if not self.is_connected_:
                self.init_i2c()

            # Leer valores del acelerometro
            acc_x = self.read_raw_data(ACCEL_ADDRESS, ACCEL_XOUT_H, True)
            acc_y = self.read_raw_data(ACCEL_ADDRESS, ACCEL_YOUT_H, True)
            acc_z = self.read_raw_data(ACCEL_ADDRESS, ACCEL_ZOUT_H, True)
            
            # Leer valores del giroscopio
            gyro_x = self.read_raw_data(GYRO_ADDRESS, GYRO_XOUT_H, True)
            gyro_y = self.read_raw_data(GYRO_ADDRESS, GYRO_YOUT_H, True)
            gyro_z = self.read_raw_data(GYRO_ADDRESS, GYRO_ZOUT_H, True)

            # Leer valores del magnetometro de la IMU
            mag_x_1 = (self.read_raw_data(MAGN_ADDRESS, MAG_XOUT_H, False) / 760.0)
            mag_y_1 = (self.read_raw_data(MAGN_ADDRESS, MAG_YOUT_H, False) / 760.0)

            self.Mag.magnetic_field.x = mag_x_1
            self.Mag.magnetic_field.y = mag_y_1
            self.mag_pub_.publish(self.Mag) 
            
            # Escalar valores segun la sensibilidad del IMU01A (suposicion: 2g y 250dps)
            self.imu_msg_.linear_acceleration.x = acc_x / 17624.0 * 9.81 # Sensibilidad para 2g
            self.imu_msg_.linear_acceleration.y = acc_y / 17624.0 * 9.81 # Multiplica 9.81 para tener m/s2
            self.imu_msg_.linear_acceleration.z = acc_z / 17624.0 * 9.81
            self.imu_msg_.angular_velocity.x = gyro_x / 131.0  # Sensibilidad para 250dps
            self.imu_msg_.angular_velocity.y = gyro_y / 131.0
            self.imu_msg_.angular_velocity.z = gyro_z / 131.0

            # Leer valores del magnetometro del GPS
            # self.read_Mag(MAGN_ADDRESS_2, MAG_XOUT_H_2, True)
            # self.mag_x /= 820.0
            # self.mag_y /= 820.0

            # mag_x = (self.mag_x + self.Xoff) * self.Xsf
            # mag_y = (self.mag_y + self.Yoff) * self.Ysf

            Th = atan2(mag_y_1,mag_x_1) - 0.0657 #- 0.5

            # self.get_logger().info("TH: %f" % Th)
            if Th < 0 :
                Th += 6.28318530718
            self.imu_msg_.orientation._z = Th

            self.imu_msg_.header.stamp = self.get_clock().now().to_msg()
            self.imu_pub_.publish(self.imu_msg_)
        except OSError:
            self.is_connected_ = False

    def init_i2c(self):
        try:
            self.bus_ = smbus.SMBus(1)
            # Configurar el sensor en modo activo
            self.bus_.write_byte_data(ACCEL_ADDRESS, CTRL_REG1, 0x2F)  # Configurar acelerometro y giroscopio a 100 Hz
            self.bus_.write_byte_data(GYRO_ADDRESS, CTRL_REG1, 0x0F)
            self.get_logger().info("Conectado al dispositivo IMU01A en la direccion %d" % GYRO_ADDRESS)
            self.bus_.write_byte_data(MAGN_ADDRESS, CRA_REG_M, 0x18)
            self.bus_.write_byte_data(MAGN_ADDRESS, CRB_REG_M, 0x40)
            self.bus_.write_byte_data(MAGN_ADDRESS, MR_REG, 0x01)
            self.get_logger().info("Conectado al dispositivo IMU01A en la direccion %d" % MAGN_ADDRESS)
            self.is_connected_ = True
        except OSError:
            self.get_logger().info("Fallo al conectar con MPU 9250 en %d" % MAGN_ADDRESS)
            self.is_connected_ = False

    def read_raw_data(self, SAD, addr, cond):
        # Leer dos bytes de datos
        if cond == True:
            low = self.bus_.read_byte_data(SAD, addr)
            high = self.bus_.read_byte_data(SAD, addr + 1)
            # Combinar valores alto y bajo
            value = ((high << 8) | low)
        else:
            high = self.bus_.read_byte_data(SAD, addr)
            low = self.bus_.read_byte_data(SAD, addr + 1)
            # Combinar valores alto y bajo
            value = ((high << 8) | low)
            
        # Convertir a valor con signo si es necesario
        if cond == True:
            if value > 32767:
                value -= 65536
        else:
            if value > 2047:
                value -= 4096
        return value
    
    def read_Mag(self, SAD, addr,cond):
        data = self.bus_.read_i2c_block_data(SAD, addr, 6)
        if cond:
            self.mag_x = (data[0] << 8) | data[1]
            self.mag_y = (data[2] << 8) | data[3]
            self.mag_z = (data[4] << 8) | data[5]
        else:
            self.mag_x = (data[1] << 8) | data[0]
            self.mag_y = (data[3] << 8) | data[2]
            self.mag_z = (data[5] << 8) | data[4]
            
        if self.mag_x > 32767: self.mag_x -= 65536
        if self.mag_y > 32767: self.mag_y -= 65536
        if self.mag_z > 32767: self.mag_z -= 65536

def main():
    rclpy.init()
    imu01a_driver = IMU01A_Driver()
    rclpy.spin(imu01a_driver)
    imu01a_driver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()