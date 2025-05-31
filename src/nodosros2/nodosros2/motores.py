import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64MultiArray
from sensor_msgs.msg import JointState
from rclpy.constants import S_TO_NS

from Phidget22.Phidget import *
from Phidget22.Devices.MotorPositionController import *
from Phidget22.Devices.BLDCMotor import *
# from tkinter import *

class Motores(Node):
    def __init__(self):
        super().__init__("Motores")
        # Nodos publicacion de comunicacion del usario
        self.pub_Vel_ = self.create_publisher(String, "velocidades/Motores", 10)
        self.wheel_cmd_sub_ = self.create_subscription(Float64MultiArray, "simple_velocity_controller/commands", self.VelocidadMotores, 10)
        self.dir_cmd_pos_sub_ = self.create_subscription(Float64MultiArray, "simple_position_controller/commands", self.CommandsCallback, 10)
        #Subscritir para iniciar navegacion
        self.new_tra_sub_ = self.create_subscription(String, 'Comandos', self.Reinicio, 10)
        self.joint_pub_ = self.create_publisher(JointState, "joint_states", 10)
        self.frequency_ = 0.01
        self.last_run = self.get_clock().now()
        self.Joints = JointState()
        self.Joints.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.Joints.velocity = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.get_logger().info("Publishing at %d Hz" % self.frequency_)

        #self.config_motores()
        #self.timer_ = self.create_timer(self.frequency_, self.EstadoMotores)

    def VelocidadMotores(self, vel):
        for i in range (0, 6):
            w = vel.data[i] # 0.1 # Convirtiendo m/s a rad/s
            ruedas = w * 9.54929659 # Convirtiendo rad/s a RPM
            ruedas = ruedas / 38.0 #Convirtiendo RPM a cte [-1, 1]
            # self.get_logger().info("Publishing at %d Hz" % w)
            # self.mvel[i].setTargetVelocity(ruedas)
    
    def EstadoMotores(self):
        dt = (self.get_clock().now() - self.last_run)
        self.Joints.header.stamp = self.get_clock().now().to_msg()
 
        for i in range (0, 6):
            # Convirtiendo de cte [-1, 1] a RPM. Motores DCM4105(38RPM)
            ruedas = self.mvel[i].getVelocity() * 38.0
            # Convirtiendo de RPM a rad/s
            ruedas = ruedas * 0.104719755
            if i <= 2:  # Ruedas Izquerda
                self.Joints.velocity[i] = -ruedas
                self.Joints.position[i] += self.Joints.velocity[i] * (dt.nanoseconds / S_TO_NS)
            else:   # Ruedas Derecha
                self.Joints.velocity[i] = ruedas
                self.Joints.position[i] += self.Joints.velocity[i] * (dt.nanoseconds / S_TO_NS)
        for i in range (0,4) :
            angulo = self.mdir[i].getPosition()# * (2*3.14159) / 1331
            self.Joints.position[i+6] = angulo
        self.joint_pub_.publish(self.Joints)
        self.last_run = self.get_clock().now()
    
    def CommandsCallback(self, msg):
        angulo = (3.14159265359 / 2.0) * 1331/(2*3.14159)
        # for i in range (0, 4):
        #     if msg.data[i] <= angulo or msg.data[i] >= -angulo:
        #         self.mdir[i].setTargetPosition(msg.data[i]) # Posiciona el motor indicado
        #         self.mdir[i].setEngaged(True)    # Habilita control de posicion

    def config_motores(self):
        self.mvel = [BLDCMotor() for i in range (0, 6)]   #Indica control de posicion
        for i in range (0, 6):
            self.mvel[i].setHubPort(i)                    #Puerto de conexion del motor al HUB
            self.mvel[i].setDeviceSerialNumber(528558)
            self.mvel[i].openWaitForAttachment(5000)

        self.mdir = [MotorPositionController() for i in range (0, 4)]   #Indica control de posicion
        for i in range (0, 4):
                self.mdir[i].setHubPort(i)                    #Puerto de conexion del motor al HUB
                self.mdir[i].setDeviceSerialNumber(539728)
                self.mdir[i].openWaitForAttachment(5000)
                self.mdir[i].setKd(-40000)                    #Constantes para control PID
                self.mdir[i].setKi(-10)
                self.mdir[i].setKp(-20000)
    
    def Reinicio(self, msg):
        if msg.data == 'A':
            self.Joints.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]


def main():
    rclpy.init()
    motor = Motores()
    rclpy.spin(motor)
    motor.mdir[0].close()
    motor.mdir[1].close()
    motor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()