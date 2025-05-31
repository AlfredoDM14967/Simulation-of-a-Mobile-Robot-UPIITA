#!/usr/bin/env python3
import rclpy.time
import smbus
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from math import atan2
from transforms3d.euler import euler2quat
from rclpy.constants import S_TO_NS

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
CRA_REG_M = 0X00      # Configuracion del magnetometro
MR_REG = 0x02         # Modo de operacion del magnetometro


class IMU01A_Driver(Node):

    def __init__(self):
        super().__init__("imu01a_driver")
        
        # Interfaz I2C
        self.is_connected_ = False
        self.init_i2c()

        # Interfaz ROS 2
        self.imu_pub_ = self.create_publisher(Imu, "/imu/out", qos_profile=qos_profile_sensor_data)
        self.new_tra_sub_ = self.create_subscription(String, 'Comandos', self.CalibrarCallback, 10)
        self.imu_msg_ = Imu()
        self.imu_msg_.header.frame_id = "base_footprint"
        self.frequency_ = 0.01
        self.auxMaxY = 0.0
        self.auxMinY = 0.0
        self.auxMaxX = 0.0
        self.auxMinX = 0.0
        self.timer_ = self.create_timer(self.frequency_, self.timerCallback)
        self.Xsf = 1.0
        self.Ysf = 69.8343
        self.Xoff = -0.0303
        self.Yoff = -24.5586

        self.calib_ = True
        self.condicion_ = True

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

            # Leer valores del magnetometro
            mag_x = (self.read_raw_data(MAGN_ADDRESS, MAG_XOUT_H, False) / 710.0) * self.Xsf + self.Xoff
            mag_y = (self.read_raw_data(MAGN_ADDRESS, MAG_YOUT_H, False) / 710.0) * self.Ysf + self.Yoff
            self.CalibRut()
            
            # Escalar valores segun la sensibilidad del IMU01A (suposicion: 2g y 250dps)
            self.imu_msg_.linear_acceleration.x = acc_x / 17624.0 * 9.81 # Sensibilidad para 2g
            self.imu_msg_.linear_acceleration.y = acc_y / 17624.0 * 9.81 # Multiplica 9.81 para tener m/s2
            self.imu_msg_.linear_acceleration.z = acc_z / 17624.0 * 9.81
            self.imu_msg_.angular_velocity.x = gyro_x / 131.0  # Sensibilidad para 250dps
            self.imu_msg_.angular_velocity.y = gyro_y / 131.0
            self.imu_msg_.angular_velocity.z = gyro_z / 131.0
            # self.imu_msg_.orientation._x = mag_x / 710.0 * 0.7791 + 0.2513
            # self.imu_msg_.orientation._y = mag_y / 710.0 * 1.2835 + 0.3524
            Th = atan2(mag_y,mag_x) - 3.85 #+ 1.57079632679
            # self.get_logger().info("Or: %f" % Th)
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
            self.get_logger().info("Conectado al dispositivo IMU01A en la direccion %d" % ACCEL_ADDRESS)
            
            # Configurar el sensor en modo activo
            self.bus_.write_byte_data(ACCEL_ADDRESS, CTRL_REG1, 0x2F)  # Configurar acelerometro y giroscopio a 100 Hz
            self.bus_.write_byte_data(GYRO_ADDRESS, CTRL_REG1, 0x0F)
            self.get_logger().info("Conectado al dispositivo IMU01A en la direccion %d" % GYRO_ADDRESS)
            self.bus_.write_byte_data(MAGN_ADDRESS, CRA_REG_M, 0x18)
            self.bus_.write_byte_data(MAGN_ADDRESS, MR_REG, 0x00)
            self.get_logger().info("Conectado al dispositivo IMU01A en la direccion %d" % MAGN_ADDRESS)
            self.is_connected_ = True
        except OSError:
            self.get_logger().info("Fallo al conectar con IMU01A en %d" % ACCEL_ADDRESS)
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
            value = ((high << 8) | low) & 0xFFF
            
        # Convertir a valor con signo si es necesario
        if cond == True:
            if value > 32767:
                value -= 65536
        else:
            if value > 2047:
                value -= 4096
        return value

    def CalibRut(self):
        if self.condicion_:
            mag_x = (self.read_raw_data(MAGN_ADDRESS, MAG_XOUT_H, False) / 710.0)
            mag_y = (self.read_raw_data(MAGN_ADDRESS, MAG_YOUT_H, False) / 710.0)
            if mag_y > self.auxMaxY:                             # Instrucciones para calibrar compas (Magnetometro).
                self.auxMaxY = mag_y
            self.get_logger().info("MaxMagY %f" % self.auxMaxY)
            if self.calib_:
                self.auxMinY = mag_y
            if mag_y < self.auxMinY:                             # Instrucciones para calibrar compas (Magnetometro).
                self.auxMinY = mag_y
            self.get_logger().info("MinMagY %f" % self.auxMinY)
            self.get_logger().info("MagY %f" % mag_y)
            if mag_x > self.auxMaxX:                             # Instrucciones para calibrar compas (Magnetometro).
                self.auxMaxX = mag_x
            self.get_logger().info("MaxMagX %f" % self.auxMaxY)
            if self.calib_:
                    self.auxMinX = mag_x                
                    self.calib_ = False
            if mag_x < self.auxMinX:                             # Instrucciones para calibrar compas (Magnetometro).
                self.auxMinX = mag_x
            self.get_logger().info("MinMagX %f" % self.auxMinX)

            if (self.auxMaxX - self.auxMinX) != 0.0 and (self.auxMaxY - self.auxMinY) != 0.0:
                self.Xsf = (self.auxMaxY - self.auxMinY) / (self.auxMaxX - self.auxMinX)
                self.Ysf = (self.auxMaxX - self.auxMinX) / (self.auxMaxY - self.auxMinY)
            if self.Xsf <= 1.0:
                self.Xsf = 1.0
            if self.Ysf <= 1.0:
                self.Ysf = 1.0
            self.Xoff = ((self.auxMaxX - self.auxMinX)/2 - self.auxMaxX) * self.Xsf
            self.Yoff = ((self.auxMaxY - self.auxMinY)/2 - self.auxMaxY) * self.Ysf
    
    def CalibrarCallback(self, msg):
        if msg.data == "Calibrar":
            self.condicion_ = False
            self.get_logger().info("Xsf  %f Ysf %f Xoff %f Y0ff %f" % (self.Xsf, self.Ysf, self.Xoff, self.Yoff))

def main():
    rclpy.init()
    imu01a_driver = IMU01A_Driver()
    rclpy.spin(imu01a_driver)
    imu01a_driver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
