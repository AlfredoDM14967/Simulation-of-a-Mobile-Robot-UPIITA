import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class STM(Node):
    def __init__(self):
        super().__init__("STM")
        # Nodos publicacion de comunicación del usario
        self.pub_STM = self.create_publisher(String, "STM", 10)
        self.counter_ = 0
        self.frequency_ = 5.0
        self.get_logger().info("Publishing at %d Hz" % self.frequency_)
        
        self.timer_ = self.create_timer(self.frequency_, self.timerCallback)

    def timerCallback(self):
        msg = String()
        msg.data = "STM: %d" % self.counter_
        self.pub_STM.publish(msg)
        self.counter_ += 1

def main():
    rclpy.init()
    simple_publisher = STM()
    rclpy.spin(simple_publisher)
    simple_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()