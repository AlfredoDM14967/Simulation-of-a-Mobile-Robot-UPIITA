#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import serial
import pynmea2
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Header

class GPSPublisher(Node):
    def __init__(self):
        super().__init__('gps_publisher')

        self.declare_parameter("port", "/dev/ttyAMA0")
        self.declare_parameter("baudrate", 9600)

        # Puerto serie del GPS
        self.port_ = self.get_parameter("port").value
        self.baudrate_ = self.get_parameter("baudrate").value
        # self.gps_serial = serial.Serial(port=self.port_, baudrate=self.baudrate_, timeout=1)

        # Crear un publisher que publicara en el topico '/gps/fix'
        self.publisher_ = self.create_publisher(NavSatFix, '/gps/fix', 10)

        # Crear un timer que llamara a la funcion 'publish_gps_data' para publicar a una taza de 75 Hz
        # self.timer = self.create_timer(0.0133, self.publish_gps_data)

    def publish_gps_data(self):
        # data_out = pynmea2.NMEAStreamReader()
        data = self.gps_serial.readline().decode('utf -8', errors='replace').strip()

        if data.startswith('$GPRMC') or data.startswith('$GNRMC'):
            msg = pynmea2.parse(data)
            
            gps_msg = NavSatFix()
            gps_msg.header = Header()
            gps_msg.header.stamp = self.get_clock().now().to_msg()
            gps_msg.header.frame_id = 'gps_link'  # Definir el frame del GPS

            if msg.status == 'A':
                gps_msg.latitude = msg.latitude
                gps_msg.longitude = msg.longitude
                gps_msg.altitude = 0.0
                # Covarianza
                gps_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN

                # Publicar el mensaje
                self.publisher_.publish(gps_msg)
                # self.get_logger().info(f"GPS Data Published: Lat: {msg.latitude}, Lon: {msg.longitude}")
            # else:
            #     self.get_logger().info("Datos No validos")

def main():
    rclpy.init()
    gps_publisher = GPSPublisher()
    rclpy.spin(gps_publisher)
    gps_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
