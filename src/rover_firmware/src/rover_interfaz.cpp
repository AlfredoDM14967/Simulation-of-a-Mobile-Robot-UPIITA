#include "/ros_ws/TT/src/rover_firmware/include/rover_firmware/rover_interfaz.hpp"
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>


namespace rover_firmware
{
RoverInterfaz::RoverInterfaz()
{
}


    RoverInterfaz::~RoverInterfaz()
    {
        int isOpen;
        for (size_t i = 0; i < 6; i++)
        {
            if (Phidget_getIsOpen((PhidgetHandle)hub1_[i], &isOpen))
            {
                try
                {
                    Phidget_close((PhidgetHandle)hub1_[i]);
                    PhidgetBLDCMotor_delete(&hub1_[i]);
                }
                catch (...)
                {
                RCLCPP_FATAL_STREAM(rclcpp::get_logger("RoverInterfaz"),
                                    "Fallo mientras cerraba hub1: " << hub1_[i]);
                }
            }
        }

        for (size_t i = 0; i < 4; i++)
        {
            if (Phidget_getIsOpen((PhidgetHandle)hub2_[i], &isOpen))
            {
                try
                {
                    Phidget_close((PhidgetHandle)hub2_[i]);
                    PhidgetMotorPositionController_delete(&hub2_[i]);
                }
                catch (...)
                {
                    RCLCPP_FATAL_STREAM(rclcpp::get_logger("RoverInterfaz"),
                                        "Fallo mientras cerraba hub2: " << hub2_[i]);
                }
            }
        }
    }


    CallbackReturn RoverInterfaz::on_init(const hardware_interface::HardwareInfo &hardware_info)
    {
        CallbackReturn result = hardware_interface::SystemInterface::on_init(hardware_info);
        if (result != CallbackReturn::SUCCESS)
        {
            return result;
        }

        port1_ = 528558;
        port2_ = 539728;

        for (size_t i = 0; i < 6; i++)
        {
            try
            {
                PhidgetBLDCMotor_create(&hub1_[i]);
                Phidget_setHubPort((PhidgetHandle)hub1_[i], i);
                Phidget_setDeviceSerialNumber((PhidgetHandle)hub1_[i], port1_);
                Phidget_openWaitForAttachment((PhidgetHandle)hub1_[i], 5000);
            }
            catch (...)
            {
                RCLCPP_FATAL_STREAM(rclcpp::get_logger("RoverInterfaz"),
                "Fallo mientras creaba hub1: " << hub1_[i]);
            }
        }

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

        velocity_commands_.reserve(info_.joints.size()); //6 ruedas para control de velocidad
        position_commands_.reserve(info_.joints.size()); //4 ruedas para control de posicion
        position_states_.reserve(info_.joints.size());  //10 ruedas para obtener posicion
        velocity_states_.reserve(info_.joints.size());  //6 ruedas para obtener velocidad

        return CallbackReturn::SUCCESS;
    }


std::vector<hardware_interface::StateInterface> RoverInterfaz::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  // Prove solo una interfaz de posicion
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_states_[i]));
    if (i<velocity_states_.size())
    {    
        state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocity_states_[i]));
    }
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> RoverInterfaz::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  // Prove solo una interfaz de velocidad
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocity_commands_[i]));
    if(i<position_commands_.size())
    {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_commands_[i]));
    }
  }

  return command_interfaces;
}

CallbackReturn RoverInterfaz::on_activate(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(rclcpp::get_logger("RoverInterfaz"), "Activando rover hardware ...");

    // Reinicia comandos y estados
    velocity_commands_ = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
    position_commands_ = { 0.0, 0.0, 0.0, 0.0 };
    position_states_ = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
    velocity_states_ = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
    RCLCPP_INFO(rclcpp::get_logger("rover_interfaz"),
                "Hardware inicializado, listo para tomar comandos!");
    return CallbackReturn::SUCCESS;
}


CallbackReturn RoverInterfaz::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("RoverInterfaz"), "Deteniendo rover hardware ...");
    int isOpen;
    for (size_t i = 0; i < 6; i++)
    {
        if (Phidget_getIsOpen((PhidgetHandle)hub1_[i], &isOpen))
        {
            try
            {
                Phidget_close((PhidgetHandle)hub1_[i]);
                PhidgetBLDCMotor_delete(&hub1_[i]);
            }
            catch (...)
            {
            RCLCPP_FATAL_STREAM(rclcpp::get_logger("RoverInterfaz"),
                                "Fallo mientras cerraba hub1: " << hub1_[i]);
            }
        }
    }

    for (size_t i = 0; i < 4; i++)
    {
        if (Phidget_getIsOpen((PhidgetHandle)hub2_[i], &isOpen))
        {
            try
            {
                Phidget_close((PhidgetHandle)hub2_[i]);
                PhidgetMotorPositionController_delete(&hub2_[i]);
            }
            catch (...)
            {
                RCLCPP_FATAL_STREAM(rclcpp::get_logger("RoverInterfaz"),
                                    "Fallo mientras cerraba hub2: " << hub2_[i]);
            }
        }
    }

  RCLCPP_INFO(rclcpp::get_logger("RoverInterfaz"), "Hardware detenido");
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type RoverInterfaz::read(const rclcpp::Time &,
                                                          const rclcpp::Duration &)
{
    auto dt = (rclcpp::Clock().now() - last_run_).seconds();
    double ruedas;

    for (size_t i = 0; i < 6; i++)
    {
        PhidgetBLDCMotor_getVelocity(hub1_[i], &ruedas);
        //Convirtiendo de cte [-1, 1] a RPM. Motores DCM4105(38RPM)
        ruedas = ruedas * 38.0;
        //Convirtiendo de RPM a rad/s
        ruedas = ruedas * 0.104719755;
        velocity_states_.at(i) = ruedas;
        position_states_.at(i) += velocity_states_.at(i) * dt;
    }

    // for (size_t i = 6; i < 10; i++)
    // {
    //     PhidgetMotorPositionController_getPosition(hub2_[i-6], &ruedas);
    //     position_states_.at(i) = ruedas;
    // }

    last_run_ = rclcpp::Clock().now();

    return hardware_interface::return_type::OK;
}


hardware_interface::return_type RoverInterfaz::write(const rclcpp::Time &,
                                                    const rclcpp::Duration &)
{  
    double w = 0;
    double ruedas = 0;
    try
  {
    for (size_t i = 0; i < 6; i++)
    {
        w = velocity_commands_.at(i); // 0.1; //Convirtiendo m/s a rad/s
        ruedas = w * 9.54929659; //Convirtiendo rad/s a RPM
        ruedas = ruedas / 38.0; //Convirtiendo RPM a cte [-1, 1]
        w = i; //Captura el numero de hub en caso de error, para mostrarlo en catch
        PhidgetBLDCMotor_setTargetVelocity(hub1_[i], ruedas);
    }
  }
  catch (...)
  {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("RoverInterfaz"),
                        "Algo fallo mientras se enviaba el mensaje "
                            << velocity_commands_.at(w) << " en el puerto " << port1_);
    return hardware_interface::return_type::ERROR;
  }

   try
  {
    for (size_t i = 0; i < 4; i++)
    {
        w = i;
        PhidgetMotorPositionController_setTargetPosition(hub2_[i], position_commands_.at(i));
        PhidgetMotorPositionController_setEngaged(hub2_[i], 1);
    }
  }
  catch (...)
  {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("RoverInterfaz"),
                        "Algo fallo mientras se enviaba el mensaje "
                            << position_commands_.at(w) << " en el puerto " << port2_);
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}
}
PLUGINLIB_EXPORT_CLASS(rover_firmware::RoverInterfaz, hardware_interface::SystemInterface)