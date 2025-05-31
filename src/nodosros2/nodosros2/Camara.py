import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Camara(Node):
    def __init__(self):
        super().__init__("Camara")
        # Nodos publicacion de comunicación del usario
        self.pub_Cam = self.create_publisher(String, "Image", 10)
        self.counter_ = 0
        self.frequency_ = 5.0
        self.get_logger().info("Publishing at %d Hz" % self.frequency_)
        
        self.timer_ = self.create_timer(self.frequency_, self.timerCallback)

    def timerCallback(self):
        msg = String()
        msg.data = "Camara: %d" % self.counter_
        self.pub_Cam.publish(msg)
        self.counter_ += 1

def main():
    rclpy.init()
    simple_publisher = Camara()
    rclpy.spin(simple_publisher)
    simple_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()