#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import TwistStamped
import numpy as np
import math

class SimpleController(Node):
    def __init__(self):
        super().__init__("simple_controller")

        self.declare_parameter("wheel_radius",0.1)
        self.declare_parameter("wheel_separation",1.015)
        self.declare_parameter("L1",0.42)

        self.wheel_radius_ = self.get_parameter("wheel_radius").get_parameter_value().double_value
        self.wheel_separation_ = self.get_parameter("wheel_separation").get_parameter_value().double_value
        self.L1_ = self.get_parameter("L1").get_parameter_value().double_value

        self.get_logger().info("Usando wheel_radius %r" % self.wheel_radius_)
        self.get_logger().info("Usando wheel_separation %r" % self.wheel_separation_)

        #publicador y subscriptor de velocidad
        self.wheel_cmd_pub_ = self.create_publisher(Float64MultiArray, "simple_velocity_controller/commands", 10)
        self.vel_sub_ = self.create_subscription(TwistStamped, "ttrobot_controller/cmd_vel", self.velCallback, 10)

        #publicador y subscriptor de posicion
        self.dir_cmd_pos_pub_ = self.create_publisher(Float64MultiArray, "simple_position_controller/commands", 10)
        self.pos_sub_ = self.create_subscription(TwistStamped, "ttrobot_controller/cmd_pos", self.velCallback, 10)

        self.speed_conversion_ = np.array([[self.wheel_radius_/2, self.wheel_radius_/2],
                                          [-self.wheel_radius_/self.wheel_separation_, self.wheel_radius_/self.wheel_separation_]])
        self.get_logger().info("La matriz de conversion es %s" % self.speed_conversion_)

    def velCallback(self, msg):
        robot_speed = np.array([[msg.twist.linear.x],
                                [msg.twist.angular.z]])
        
        wheel_speed = np.matmul(np.linalg.inv(self.speed_conversion_), robot_speed)
        wheel_speed_msg = Float64MultiArray()
        wheel_speed_msg.data = [wheel_speed[0, 0], -wheel_speed[1, 0], wheel_speed[0, 0],
                                -wheel_speed[1, 0], wheel_speed[0, 0], -wheel_speed[1, 0]]
        self.wheel_cmd_pub_.publish(wheel_speed_msg)

        #Calculo del angulo de la direccion
        if (wheel_speed[1,0]-wheel_speed[0,0])==0.0:
            alpha_ = 0.0
        else :
            ICRl_ = wheel_speed[0,0]*self.wheel_separation_/(wheel_speed[1,0]-wheel_speed[0,0])
            alpha_ = math.atan2(self.L1_,ICRl_+self.wheel_separation_/2)
            
        angulo_direc_msg = Float64MultiArray()
        if robot_speed[0,0]==0:
            #Radianes que gira el robot para formar la trayectoria planeada
            angulo_direc_msg.data = [-0.785398,0.785398,0.785398,-0.785398]
        else:
            angulo_direc_msg.data = [-alpha_,alpha_,-alpha_,alpha_]
        self.dir_cmd_pos_pub_.publish(angulo_direc_msg)

def main():
    rclpy.init()
    simple_controller = SimpleController()
    rclpy.spin(simple_controller)
    simple_controller.destroy_node()
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()