#!/usr/bin/env python3
import rclpy.time
import smbus
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, MagneticField
from math import atan2
from tf_transformations import quaternion_from_euler

# Direcciones y registros del HW-290 (actualiza con los valores correctos del HW-290)
HW290_ADDRESS = 0x68  # Dirección I2C del HW-290
ACCEL_XOUT_H = 0x3B   # Registro de salida de acelerómetro (alto)
GYRO_XOUT_H = 0x43    # Registro de salida de giroscopio (alto)
MAG_XOUT_H = 0x49     # Registro de salida de magnetómetro (alto, si aplica)
CONFIG = 0x1A         # Registro de configuración
PWR_MGMT_1 = 0x6B     # Registro de gestión de energía
MAG_CONFIG = 0x0A     # Registro de configuración del magnetómetro

class HW290Driver(Node):
    def __init__(self):
        super().__init__("hw290_driver")
        
        # Interfaz I2C
        self.is_connected_ = False
        self.init_i2c()

        # Interfaz ROS 2
        self.imu_pub_ = self.create_publisher(Imu, "/imu/out", qos_profile=qos_profile_sensor_data)
        self.mag_pub_ = self.create_publisher(MagneticField, "/mag/out", qos_profile=qos_profile_sensor_data)
        self.imu_msg_ = Imu()
        self.imu_msg_.header.frame_id = "base_footprint"
        self.mag_msg_ = MagneticField()
        self.mag_msg_.header.frame_id = "base_footprint"
        self.frequency_ = 0.01
        self.timer_ = self.create_timer(self.frequency_, self.timerCallback)

    def timerCallback(self):
        try:
            if not self.is_connected_:
                self.init_i2c()

            # Leer valores de acelerómetro y giroscopio
            acc_x = self.read_raw_data(ACCEL_XOUT_H)
            acc_y = self.read_raw_data(ACCEL_XOUT_H + 2)
            acc_z = self.read_raw_data(ACCEL_XOUT_H + 4)

            gyro_x = self.read_raw_data(GYRO_XOUT_H)
            gyro_y = self.read_raw_data(GYRO_XOUT_H + 2)
            gyro_z = self.read_raw_data(GYRO_XOUT_H + 4)

            # Leer valores del magnetómetro
            mag_x = self.read_raw_data(MAG_XOUT_H)
            mag_y = self.read_raw_data(MAG_XOUT_H + 2)
            mag_z = self.read_raw_data(MAG_XOUT_H + 4)

            # Escalar valores según la sensibilidad del HW-290 (actualiza con los valores correctos)
            self.imu_msg_.linear_acceleration.x = acc_x / 16384.0 * -9.81
            self.imu_msg_.linear_acceleration.y = acc_y / 16384.0 * -9.81
            self.imu_msg_.linear_acceleration.z = acc_z / 16384.0 * -9.81

            self.imu_msg_.angular_velocity.x = gyro_x / 131.0
            self.imu_msg_.angular_velocity.y = gyro_y / 131.0
            self.imu_msg_.angular_velocity.z = gyro_z / 131.0

            self.mag_msg_.magnetic_field.x = mag_x * 0.15 
            self.mag_msg_.magnetic_field.y = mag_y * 0.15
            self.mag_msg_.magnetic_field.z = mag_z * 0.15

            self.imu_msg_.header.stamp = self.get_clock().now().to_msg()
            self.mag_msg_.header.stamp = self.get_clock().now().to_msg()

            self.imu_pub_.publish(self.imu_msg_)
            self.mag_pub_.publish(self.mag_msg_)
        except OSError:
            self.is_connected_ = False

    def init_i2c(self):
        try:
            self.bus_ = smbus.SMBus(1)
            # Configuracion del HW-290
            self.bus_.write_byte_data(HW290_ADDRESS, PWR_MGMT_1, 0x00)  # Activar dispositivo
            self.bus_.write_byte_data(HW290_ADDRESS, CONFIG, 0x01)
            self.bus_.write_byte_data(HW290_ADDRESS, MAG_CONFIG, 0x16) # Configurar magnetómetro (modo continuo)
            self.get_logger().info("Conectado al dispositivo HW-290 en la dirección %d" % HW290_ADDRESS)
            self.is_connected_ = True
        except OSError:
            self.get_logger().info("Fallo al conectar con HW-290 en %d" % HW290_ADDRESS)
            self.is_connected_ = False

    def read_raw_data(self, addr):
        high = self.bus_.read_byte_data(HW290_ADDRESS, addr)
        low = self.bus_.read_byte_data(HW290_ADDRESS, addr + 1)
        value = (high << 8) | low
        if value > 32767:
            value -= 65536
        return value

def main():
    rclpy.init()
    hw290_driver = HW290Driver()
    rclpy.spin(hw290_driver)
    hw290_driver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
