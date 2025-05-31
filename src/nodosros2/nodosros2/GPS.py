import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class GPS(Node):
    def __init__(self):
        super().__init__("GPS")
        # Nodos publicacion de comunicación del usario
        self.pub_GPS = self.create_publisher(String, "NavStatFix", 10)
        self.pub_Mg = self.create_publisher(String, "MagneticField", 10)
        self.counter_ = 0
        self.frequency_ = 5.0
        self.get_logger().info("Publishing at %d Hz" % self.frequency_)
        
        self.timer_ = self.create_timer(self.frequency_, self.timerCallback)
        self.timer_2 = self.create_timer(self.frequency_, self.MagCallback)

    def timerCallback(self):
        msg = String()
        msg.data = "GPS: %d" % self.counter_
        self.pub_GPS.publish(msg)

    def MagCallback(self):
        msg = String()
        msg.data = "Magnetometro: %d" % 3.347*self.counter_
        self.pub_Mg.publish(msg)
        self.counter_ += 1


def main():
    rclpy.init()
    simple_publisher = GPS()
    rclpy.spin(simple_publisher)
    simple_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()