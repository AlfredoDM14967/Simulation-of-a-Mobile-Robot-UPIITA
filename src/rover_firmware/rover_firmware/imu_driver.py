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

MAGN_ADDRESS_2 = 0x1E   #Direccion magnetometr HMC5883L 0x1E
CONF_A = 0x00
CONF_B = 0x01
MODE_REG = 0x02
MAG_XOUT_H_2 = 0x03     # Registro de salida de magnetometro del GPS (alto)
MAG_ZOUT_H_2 = 0x05     # Registro de salida de magnetometro del GPS (alto)
MAG_YOUT_H_2 = 0x07     # Registro de salida de magnetometro del GPS (alto)



# Direcciones para IMU HW-290
IMU_ADDRESS = 0x68      
G_CONFIG_1 = 0X1A
G_CONFIG_2 = 0X1B
A_CONFIG_1 = 0X1C
A_CONFIG_2 = 0X1D
USERCTRL = 0x6A
PWM_MGM_1 = 0x6B
I2CMASTERCTRL = 0x24

PWM_MGM_1 = 0x6B
G_CONFIG_1 = 0X1A
G_CONFIG_2 = 0X1B
A_CONFIG_1 = 0X1C
A_CONFIG_2 = 0X1D
INT_PIN_CFG = 0x37

ACCEL_XOUT_H = 0X3B
ACCEL_YOUT_H = 0X3D
ACCEL_ZOUT_H = 0X3F
GYRO_XOUT_H = 0x43
GYRO_YOUT_H = 0x45
GYRO_ZOUT_H = 0x47

# Direcciones para IMU MPU 9250
IMU_ADDRESS_MPU = 0x69    
AK8963_ADDR = 0x0C
INT_PIN_CFG_2 = 0x37
CONFIG = 0x1A
GYRO_CONFIG = 0x1B
ACCEL_CONFIG = 0x1C
ACCEL_CONFIG2 =0x1D

ACCEL_XOUT_H_MPU = 0X3B
ACCEL_YOUT_H_MPU = 0X3D
ACCEL_ZOUT_H_MPU = 0X3F
GYRO_XOUT_H_MPU = 0x43
GYRO_YOUT_H_MPU = 0x45
GYRO_ZOUT_H_MPU = 0x47
AK8963_ST1 = 0x02
MAG_XOUT_L = 0X03
MAG_YOUT_L = 0X05
MAG_ZOUT_L = 0X07
AK8963_CNTL = 0x0A
AK8963_ASAX = 0x10
AK8963_ASAY = 0x11
AK8963_XOUT_L = 0x03

class IMU01A_Driver(Node):
    def __init__(self):
        super().__init__("MPU9250_driver")
        
        # Interfaz I2C
        self.is_connected_ = False
        # self.init_i2c()

        # Interfaz ROS 2
        self.imu_pub_ = self.create_publisher(Imu, "/imu/out", qos_profile=qos_profile_sensor_data)
        self.mag_pub_ = self.create_publisher(MagneticField, "/mag/out", qos_profile=qos_profile_sensor_data)
        self.imu_msg_ = Imu()
        self.imu_msg_.header.frame_id = "base_footprint"
        self.frequency_ = 0.01
        self.auxMaxY = 0.0
        self.auxMinY = 0.0
        self.auxMaxX = 0.0
        self.auxMinX = 0.0
        # self.timer_ = self.create_timer(self.frequency_, self.timerCallback)
        # Valores de calibracion del MPU
        self.Xsf = 0.9780 #0.9876
        self.Ysf = 1.0230 #1.0127
        self.Xoff = 38 #-3.0
        self.Yoff = 125 #114.0

        # Valores de calibracion del HMC5883L
        self.Xsf_2 =  1.0269 #1.0218 
        self.Ysf_2 =  0.9745 #0.9791 
        self.Xoff_2 = 0.0659 #0.0829
        self.Yoff_2 = -0.0256 #-0.0366
        self.ASA_X = 0.0   # Valores para ajustar sensibilidad del magnetometro
        self.ASA_Y = 0.0

        self.Mag = MagneticField()

    def timerCallback(self):
        try:
            if not self.is_connected_:
                self.init_i2c()

            # Leer valores de la IMU HW-290
            acc_x = self.read_raw_data(IMU_ADDRESS, ACCEL_XOUT_H, True)
            acc_y = self.read_raw_data(IMU_ADDRESS, ACCEL_YOUT_H, True)
            acc_z = self.read_raw_data(IMU_ADDRESS, ACCEL_ZOUT_H, True)
            gyro_x = self.read_raw_data(IMU_ADDRESS, GYRO_XOUT_H, True)
            gyro_y = self.read_raw_data(IMU_ADDRESS, GYRO_YOUT_H, True)
            gyro_z = self.read_raw_data(IMU_ADDRESS, GYRO_ZOUT_H, True)

            # Leer valores del magnetometro HMC5883L
            self.mag_x_2 = self.read_raw_data(MAGN_ADDRESS_2, MAG_XOUT_H_2, True)
            self.mag_y_2 = self.read_raw_data(MAGN_ADDRESS_2, MAG_YOUT_H_2, True)
            self.mag_z_2 = self.read_raw_data(MAGN_ADDRESS_2, MAG_ZOUT_H_2, True)
            self.mag_x_2 /= 820.0
            self.mag_y_2 /= 820.0

            # Leer valores del MPU-9250
            acc_x_2 = self.read_raw_data(IMU_ADDRESS_MPU,ACCEL_XOUT_H_MPU, True)
            acc_y_2 = self.read_raw_data(IMU_ADDRESS_MPU,ACCEL_YOUT_H_MPU, True)
            acc_z_2 = self.read_raw_data(IMU_ADDRESS_MPU,ACCEL_ZOUT_H_MPU, True)
            gyro_x_2 = self.read_raw_data(IMU_ADDRESS_MPU, GYRO_XOUT_H_MPU, True)
            gyro_y_2 = self.read_raw_data(IMU_ADDRESS_MPU, GYRO_YOUT_H_MPU, True)
            gyro_z_2 = self.read_raw_data(IMU_ADDRESS_MPU, GYRO_ZOUT_H_MPU, True)
            
            # Leer AK8963
            self.read_Mag(AK8963_ADDR, AK8963_ST1, False)

            # self.Mag.magnetic_field.x = float(self.mag_x_2)   # Magx es MPU, Magx2 es HMC
            # self.Mag.magnetic_field.y = float(self.mag_y_2)
            # self.mag_pub_.publish(self.Mag)

            mag_x = (self.mag_x + self.Xoff) * self.Xsf
            mag_x_2 = (self.mag_x_2 + self.Xoff_2) * self.Xsf_2
            mag_y = (self.mag_y + self.Yoff) * self.Ysf
            mag_y_2 = (self.mag_y_2 + self.Yoff_2) * self.Ysf_2
            
            # Escalar valores segun la sensibilidad del HW-290 (suposicion: 2g y 250dps)
            self.imu_msg_.linear_acceleration.x = (acc_x_2 + acc_x) / 2 / 16590.0 * -9.81 # Sensibilidad para 2g
            self.imu_msg_.linear_acceleration.y = (acc_y_2 + acc_y) / 2 / 16658.0 * -9.81 # Multiplica 9.81 para tener m/s2
            self.imu_msg_.linear_acceleration.z = (acc_z_2 + acc_z) / 2 / 16620.0 * -9.81
            self.imu_msg_.angular_velocity.x = (gyro_x_2 + gyro_x) / 2 / 131.0  # Sensibilidad para 250dps
            self.imu_msg_.angular_velocity.y = (gyro_y_2 + gyro_y) / 2 / 131.0
            self.imu_msg_.angular_velocity.z = (gyro_z_2 + gyro_z) / 2 / 131.0

            Th = atan2(mag_x,mag_y) - 0.0777
            if Th < 0:
                Th -= 0.04
            else:
                Th += 0.04   
            Th_2 = atan2(mag_y_2,mag_x_2) - 0.0777
            Th_3 = (Th + Th_2) / 2

            # self.get_logger().info("TH: %f" % Th)
            # self.get_logger().info("TH2: %f" % Th_2)
            if Th_3 < 0 :
                Th_3 += 6.28318530718
            self.imu_msg_.orientation._z = Th_3

            self.imu_msg_.header.stamp = self.get_clock().now().to_msg()
            self.imu_pub_.publish(self.imu_msg_)
        except OSError:
            self.is_connected_ = False

    def init_i2c(self):
        try:
            self.bus_ = smbus.SMBus(1)

            # Configuracion HW-290
            self.bus_.write_byte_data(IMU_ADDRESS, G_CONFIG_1, 0x00)
            self.bus_.write_byte_data(IMU_ADDRESS, G_CONFIG_2, 0x00)
            self.bus_.write_byte_data(IMU_ADDRESS, A_CONFIG_1, 0x00)
            self.bus_.write_byte_data(IMU_ADDRESS, A_CONFIG_2, 0x00)
            # Habilitar bypass I2C para el magnetómetro
            self.bus_.write_byte_data(IMU_ADDRESS, PWM_MGM_1, 0x00)
            self.bus_.write_byte_data(IMU_ADDRESS, INT_PIN_CFG, 0x02)
            self.get_logger().info("Conectado al dispositivo HW-290 en la direccion %d" % IMU_ADDRESS)

            # Configuracion HMC5883L
            self.bus_.write_byte_data(MAGN_ADDRESS_2, CONF_A, 0x70)
            self.bus_.write_byte_data(MAGN_ADDRESS_2, CONF_B, 0x40)
            self.bus_.write_byte_data(MAGN_ADDRESS_2, MODE_REG, 0x00)
            self.get_logger().info("Conectado al dispositivo HMC5883L en la direccion %d" % MAGN_ADDRESS_2)

            # Configuracion MPU 9250
            self.bus_.write_byte_data(IMU_ADDRESS_MPU, GYRO_CONFIG, 0x00)
            self.bus_.write_byte_data(IMU_ADDRESS_MPU, ACCEL_CONFIG, 0x00)
            self.bus_.write_byte_data(IMU_ADDRESS_MPU, ACCEL_CONFIG2, 0x00)
            self.bus_.write_byte_data(IMU_ADDRESS_MPU, INT_PIN_CFG_2, 0x02)

            # Configuracion AK8963
            self.bus_.write_byte_data(AK8963_ADDR, AK8963_CNTL, 0x00)  # Modo de reposo
            sleep(0.01)
            self.bus_.write_byte_data(AK8963_ADDR, AK8963_CNTL, 0x0F)  # Modo FUSE ROM acceso
            sleep(0.01)
            # Obtener valores de sensibilidad del magnetometro
            ASA = self.bus_.read_i2c_block_data(AK8963_ADDR,AK8963_ASAX,3)
            self.ASA_X = (ASA[0] - 128.0) / 256.0 + 1.0
            self.ASA_Y = (ASA[1] - 128.0) / 256.0 + 1.0
            self.get_logger().info("%f" % self.ASA_X)
            # Inicializar el AK8963 en modo continuo (16 bits, modo 100 Hz)
            self.bus_.write_byte_data(AK8963_ADDR, AK8963_CNTL, 0x16)
            self.get_logger().info("Conectado al dispositivo AK8993 en la direccion %d" % AK8963_ADDR)
            self.is_connected_ = True
        except OSError:
            self.get_logger().info("Fallo al conectar con MPU 9250 en %d" % IMU_ADDRESS)
            self.is_connected_ = False

    def read_raw_data(self, SAD, addr, cond):
        # Leer dos bytes de datos
        if cond == True:
            high = self.bus_.read_byte_data(SAD, addr)
            low = self.bus_.read_byte_data(SAD, addr + 1)
            # Combinar valores alto y bajo
            value = ((high << 8) | low)
        else:
            low = self.bus_.read_byte_data(SAD, addr)
            high = self.bus_.read_byte_data(SAD, addr + 1)
            # Combinar valores alto y bajo
            value = ((high << 8) | low)
            
        # Convertir a valor con signo si es necesario
        if value > 32767:
            value -= 65536

        return value
    
    def read_Mag(self, SAD, addr,cond):
        data = self.bus_.read_i2c_block_data(SAD, addr, 8)
        if cond:
            self.mag_x = (data[0] << 8) | data[1]
            self.mag_y = (data[2] << 8) | data[3]
            self.mag_z = (data[4] << 8) | data[5]
        else:
            self.mag_x = (data[2] << 8) | data[1]
            self.mag_y = (data[4] << 8) | data[3]
            self.mag_z = (data[6] << 8) | data[5]
            
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