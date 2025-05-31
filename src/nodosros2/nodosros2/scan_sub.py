import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
import math

class SubLidar(Node):
    def __init__(self):
        super().__init__("LidarSub")
        self.pub_Lidar_ = self.create_publisher(String, "/LIDAR", 10)
        self.sub_Lidar_ = self.create_subscription(LaserScan, "scan", self.ScanCallback, 10)
        self.pointsF = 0    # Contador de puntos obstaculo frontal
        self.pointsD = 0    # Contador de puntos obstaculo lateral derecha
        self.pointsI = 0    # Contador de puntos obstaculo lateral izquierda

    def ScanCallback(self, msg):
        count = msg.scan_time / msg.time_increment
        for i in range (0, int(count)):
            grad = 57.296 * (msg.angle_min + msg.angle_increment * i)
            X = -1.0 * msg.ranges[i] * math.cos(grad * 0.0174533)
            if (grad > 130.0 or grad < -130.0):
                if (X < 0.80):
                    self.pointsF += 1
                    # self.get_logger().info("Distancia %f m Angulo %f" % (X , grad))
            Y = abs(msg.ranges[i] * math.sin(grad * 0.0174533))
            if (grad > 30.0 and grad < 95.0):
                if (Y < 1.5):
                    self.pointsD += 1
                    # self.get_logger().info("Distancia %f m Angulo %f" % (Y , grad))
            if (grad > -95.0 and grad < -30.0):
                if (Y < 1.5):
                    self.pointsI += 1

        if self.pointsF >= 80 :
            # self.get_logger().info("Obstaculo Frontal! %d" % self.pointsF)
            self.pointsF = 0
            F = "1 "
        else :
            F = "0 "
        if self.pointsD >= 80 :
            # self.get_logger().info("Obstaculo Lateral Derecha! %d" % self.pointsD)
            self.pointsD = 0
            D = "1 "
        else :
            D = "0 "
        if self.pointsI >= 80 :
            # self.get_logger().info("Obstaculo Lateral Izquierda! %d" % self.pointsI)
            self.pointsI = 0
            I = "1"
        else :
            I = "0"
        obs = String()
        obs.data = F + D + I
        self.pub_Lidar_.publish(obs)

def main():
    rclpy.init()
    lidar = SubLidar()
    rclpy.spin(lidar)
    lidar.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()