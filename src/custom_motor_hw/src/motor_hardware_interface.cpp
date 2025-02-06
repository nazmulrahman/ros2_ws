#include <hardware_interface/system_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <serial/serial.h>

using hardware_interface::CallbackReturn;
using hardware_interface::StateInterface;
using hardware_interface::CommandInterface;
using hardware_interface::return_type;

class MotorHardwareInterface : public hardware_interface::SystemInterface
{
private:
    serial::Serial serial_conn_;
    std::vector<double> hw_commands_;
    std::vector<double> hw_states_;

public:
    // Initialize hardware
    CallbackReturn on_init(const hardware_interface::HardwareInfo &info)
    {
        if (SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
            return CallbackReturn::ERROR;

        hw_commands_.resize(info.joints.size(), 0.0);
        hw_states_.resize(info.joints.size(), 0.0);

        RCLCPP_INFO(rclcpp::get_logger("MotorHardwareInterface"), "Starting hardware interface...");

        // Open Serial Connection
        try
        {
            serial_conn_.setPort("/dev/ttyACM0");
            serial_conn_.setBaudrate(9600);
            serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
            serial_conn_.setTimeout(timeout);
            serial_conn_.open();
            RCLCPP_INFO(rclcpp::get_logger("MotorHardwareInterface"), "Successfully opened serial port: /dev/ttyACM0");
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(rclcpp::get_logger("MotorHardwareInterface"), "Failed to open serial port: %s", e.what());
            return CallbackReturn::ERROR;
        }

        return CallbackReturn::SUCCESS;
    }

    // Export joint state interfaces
    std::vector<StateInterface> export_state_interfaces() override
    {
        std::vector<StateInterface> state_interfaces;
        state_interfaces.emplace_back("left_wheel_joint", hardware_interface::HW_IF_VELOCITY, &hw_states_[0]);
        state_interfaces.emplace_back("right_wheel_joint", hardware_interface::HW_IF_VELOCITY, &hw_states_[1]);
        return state_interfaces;
    }


    // Export joint command interfaces
    std::vector<CommandInterface> export_command_interfaces() override
    {
        std::vector<CommandInterface> command_interfaces;
        command_interfaces.emplace_back("left_wheel_joint", hardware_interface::HW_IF_VELOCITY, &hw_commands_[0]);
        command_interfaces.emplace_back("right_wheel_joint", hardware_interface::HW_IF_VELOCITY, &hw_commands_[1]);
        return command_interfaces;
    }


    // Read current wheel speeds from Arduino
    return_type read(const rclcpp::Time &, const rclcpp::Duration &) override
    {
        if (serial_conn_.available())
        {
            std::string response = serial_conn_.readline();
            sscanf(response.c_str(), "%lf,%lf", &hw_states_[0], &hw_states_[1]);
        }
        return return_type::OK;
    }

    // Send velocity commands to Arduino
    return_type write(const rclcpp::Time &, const rclcpp::Duration &) override
    {
        char cmd[50];
        snprintf(cmd, sizeof(cmd), "%d,%d\n",
                 static_cast<int>(hw_commands_[0] * 255),
                 static_cast<int>(hw_commands_[1] * 255));
        serial_conn_.write(cmd);
        return return_type::OK;
    }
};

// Register the plugin
PLUGINLIB_EXPORT_CLASS(MotorHardwareInterface, hardware_interface::SystemInterface)
