import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Controller_manager(Node):
    def __init__(self):
        super().__init__("Controller_manager")
        # Nodos publicacion de comunicación del usario
        self.pub_CM = self.create_publisher(String, "Controller_manager", 10)
        self.counter_ = 0
        self.frequency_ = 5.0
        self.get_logger().info("Publishing at %d Hz" % self.frequency_)
        
        self.timer_ = self.create_timer(self.frequency_, self.timerCallback)

    def timerCallback(self):
        msg = String()
        msg.data = "Controller Manager: %d" % self.counter_
        self.pub_CM.publish(msg)
        self.counter_ += 1

def main():
    rclpy.init()
    simple_publisher = Controller_manager()
    rclpy.spin(simple_publisher)
    simple_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()