#include <rclcpp/rclcpp.hpp>

#include <vector>
#include <string>

#include <stdlib.h>
#include <phidget22.h>
#include <stdio.h>


using namespace std::chrono_literals;

class SimplePublisher : public rclcpp::Node
{
public:
  SimplePublisher() : Node("simple_publisher")
  {
    timer_ = create_wall_timer(1s, std::bind(&SimplePublisher::timerCallback, this));
    RCLCPP_INFO(get_logger(), "Publishing at 1 Hz");
    PhidgetBLDCMotorHandle hub1_[6];
    double port1_;
    PhidgetMotorPositionControllerHandle hub2_[4];
    double port2_;
    port1_ = 528558;
    port2_ = 539728;
    for (size_t i = 0; i < 4; i++)
        {
            try
            {
                PhidgetMotorPositionController_create(&hub2_[i]);
                Phidget_setHubPort((PhidgetHandle)hub2_[i], i);
                Phidget_setDeviceSerialNumber((PhidgetHandle)hub2_[i], port2_);
                Phidget_openWaitForAttachment((PhidgetHandle)hub2_[i], 5000);
                PhidgetMotorPositionController_setKd(hub2_[i], -40000);
                PhidgetMotorPositionController_setKp(hub2_[i], -5);
                PhidgetMotorPositionController_setKi(hub2_[i], -20000);
                //Factor de escala para pasar de cte 1331 a rad
                PhidgetMotorPositionController_setRescaleFactor(hub2_[i], 1331/(2*3.14159));
            }
            catch (...)
            {
                RCLCPP_FATAL_STREAM(rclcpp::get_logger("RoverInterfaz"),
                "Fallo mientras creaba hub2: " << hub2_[i]);
            }
        }
  }

  void timerCallback()
  {
    
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
};


int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SimplePublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}