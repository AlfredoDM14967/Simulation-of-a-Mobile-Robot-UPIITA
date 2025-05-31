#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import socket

class SimpleSerialTransmitter(Node):
    def __init__(self):
        super().__init__("simple_serial_transmitter")

        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baudrate", 115200)

        self.port_ = self.get_parameter("port").value
        self.baudrate_ = self.get_parameter("baudrate").value

        self.sub_ = self.create_subscription(String, "Comandos", self.msgCallback, 10)
        self.arduino_ = serial.Serial(port=self.port_, baudrate=self.baudrate_, timeout=0.1)
        self.ip = "0.0.0.0"

    def msgCallback(self, msg):
        self.get_logger().info("New message received, publishing on serial: %s" % self.arduino_.name)
        self.arduino_.write(msg.data.encode("utf-8"))
    
    def IpCallbak(self):
        # Función para obtener la dirección IP
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(("8.8.8.8", 80))
            self.ip = s.getsockname()[0]
            s.close()
            IP = String()
            IP.data = "i" + str(self.ip)
            self.msgCallback(IP)
        except:
            self.ip = 'No IP'
        return self.ip


def main():
    rclpy.init()

    simple_serial_transmitter = SimpleSerialTransmitter()
    rclpy.spin(simple_serial_transmitter)
    
    simple_serial_transmitter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
