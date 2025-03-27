#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "robot_hw/arduino_comms.hpp"

using std::placeholders::_1;

class MotorControllerPID : public rclcpp::Node {
public:
    MotorControllerPID()
        : Node("motor_controller_pid"), prev_time_(this->now()) {
        
        // Parameters
        this->declare_parameter<std::string>("serial_device", "/dev/ttyACM0");
        this->declare_parameter<int>("baud_rate", 57600);
        this->declare_parameter<int>("timeout_ms", 1000);
        this->declare_parameter<double>("wheel_separation", 0.36);
        this->declare_parameter<double>("wheel_radius", 0.065);
        this->declare_parameter<double>("Kp", 1.0);
        this->declare_parameter<double>("Ki", 0.1);
        this->declare_parameter<double>("Kd", 0.05);
        
        // Get parameters
        serial_device_ = this->get_parameter("serial_device").as_string();
        baud_rate_ = this->get_parameter("baud_rate").as_int();
        timeout_ms_ = this->get_parameter("timeout_ms").as_int();
        wheel_separation_ = this->get_parameter("wheel_separation").as_double();
        wheel_radius_ = this->get_parameter("wheel_radius").as_double();
        Kp_ = this->get_parameter("Kp").as_double();
        Ki_ = this->get_parameter("Ki").as_double();
        Kd_ = this->get_parameter("Kd").as_double();
        
        ms_to_rpm_ = (1/(wheel_radius_ * 2 * 3.1415)) * 60;
        
        // Initialize Arduino communication
        RCLCPP_INFO(this->get_logger(), "Connecting to Arduino...");
        arduino_.connect(serial_device_, baud_rate_, timeout_ms_);
        
        if (!arduino_.connected()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to connect to Arduino!");
            rclcpp::shutdown();
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Connected successfully!");

        arduino_.send_empty_msg();

        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10, std::bind(&MotorControllerPID::cmd_vel_callback, this, _1));
        ticks_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(
            "ticks", 10, std::bind(&MotorControllerPID::ticks_callback, this, _1));
    }

    ~MotorControllerPID() {
        RCLCPP_INFO(this->get_logger(), "Disconnecting from Arduino...");
        arduino_.disconnect();
        RCLCPP_INFO(this->get_logger(), "Disconnected successfully.");
    }

private:
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        target_left_rpm_ = (msg->linear.x - (msg->angular.z * wheel_separation_ / 2)) * ms_to_rpm_;
        target_right_rpm_ = (msg->linear.x + (msg->angular.z * wheel_separation_ / 2)) * ms_to_rpm_;
    }

    void ticks_callback(const geometry_msgs::msg::Vector3::SharedPtr msg) {
        rclcpp::Time now = this->now();
        double dt = (now - prev_time_).seconds();
        prev_time_ = now;
        
        if (dt == 0) return;

        // Convert ticks to RPM
        double actual_left_rpm = (msg->x / dt) * 60.0 / encoder_ticks_per_rev_;
        double actual_right_rpm = (msg->y / dt) * 60.0 / encoder_ticks_per_rev_;

        // Compute PID control
        int left_pwm = compute_pid(target_left_rpm_, actual_left_rpm, left_error_, left_integral_, left_prev_error_, dt);
        int right_pwm = compute_pid(target_right_rpm_, actual_right_rpm, right_error_, right_integral_, right_prev_error_, dt);
        
        // Send commands
        if (!arduino_.connected()) {
            RCLCPP_ERROR(this->get_logger(), "Lost connection to Arduino!");
            return;
        }
        arduino_.set_motor_values_ol(left_pwm, right_pwm);
    }

    int compute_pid(double target_rpm, double actual_rpm, double &error, double &integral, double &prev_error, double dt) {
        error = target_rpm - actual_rpm;
        integral += error * dt;
        double derivative = (error - prev_error) / dt;
        prev_error = error;
        
        int pwm = static_cast<int>(Kp_ * error + Ki_ * integral + Kd_ * derivative);
        return std::clamp(pwm, -255, 255);
    }

    std::string serial_device_;
    int baud_rate_, timeout_ms_;
    double wheel_radius_, wheel_separation_;
    double Kp_, Ki_, Kd_;
    double ms_to_rpm_;
    double encoder_ticks_per_rev_ = 160.0; // Adjust according to encoder resolution
    
    double target_left_rpm_ = 0.0, target_right_rpm_ = 0.0;
    double left_error_ = 0.0, right_error_ = 0.0;
    double left_integral_ = 0.0, right_integral_ = 0.0;
    double left_prev_error_ = 0.0, right_prev_error_ = 0.0;
    
    rclcpp::Time prev_time_;
    robot_hw::ArduinoComms arduino_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr ticks_sub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MotorControllerPID>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}