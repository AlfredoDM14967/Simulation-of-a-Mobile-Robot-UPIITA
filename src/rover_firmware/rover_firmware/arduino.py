#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial

class SimpleSerialTransmitter(Node):
    def __init__(self):
        super().__init__("simple_serial_transmitter")

        self.declare_parameter("port", "/dev/ttyUSB1")
        self.declare_parameter("baudrate", 9600)

        self.port_ = self.get_parameter("port").value
        self.baudrate_ = self.get_parameter("baudrate").value

        self.pub_ = self.create_publisher(String, "Comandos", 10)
        self.sub_ = self.create_subscription(String, "Comandos", self.msgCallback, 10)
        # self.arduino_ = serial.Serial(port=self.port_, baudrate=self.baudrate_, timeout=0.1)
        self.ip = "0.0.0.0"

        self.frequency_ = 0.01
        # self.timer_ = self.create_timer(self.frequency_, self.timerCallback)

    def msgCallback(self, msg):
        aux = str(msg.data).split()
        # if aux[0] != 'v':
        #     # self.get_logger().info("New message received, publishing on serial: %s" % msg.data)
        #     self.arduino_.write(msg.data.encode("utf-8"))
    
    def timerCallback(self):
        if rclpy.ok() and self.arduino_.is_open:
            data = self.arduino_.readline()
            aux = 0.0
            try:
                data.decode("utf-8").strip()
                aux = float(data)
            except:
                return
            
            msg = String()
            msg.data = 'v ' + str(aux)
            # self.get_logger().info("%s" % msg.data)
            self.pub_.publish(msg)


def main():
    rclpy.init()

    simple_serial_transmitter = SimpleSerialTransmitter()
    rclpy.spin(simple_serial_transmitter)
    
    simple_serial_transmitter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
