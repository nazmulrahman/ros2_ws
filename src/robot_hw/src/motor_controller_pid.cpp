#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "geometry_msgs/msg/twist.hpp"

class MotorControllerPID : public rclcpp::Node {
public:
    MotorControllerPID() : Node("motor_controller_pid") {
        // Declare and get initial PID parameters
        this->declare_parameter("Kp", 1.0);
        this->declare_parameter("Ki", 0.0);
        this->declare_parameter("Kd", 0.0);

        Kp_ = this->get_parameter("Kp").as_double();
        Ki_ = this->get_parameter("Ki").as_double();
        Kd_ = this->get_parameter("Kd").as_double();

        // Setup dynamic parameter callback
        this->add_on_set_parameters_callback(
            [this](const std::vector<rclcpp::Parameter> &params) {
                for (const auto &param : params) {
                    if (param.get_name() == "Kp") Kp_ = param.as_double();
                    if (param.get_name() == "Ki") Ki_ = param.as_double();
                    if (param.get_name() == "Kd") Kd_ = param.as_double();
                }
                RCLCPP_INFO(this->get_logger(), "Updated PID: Kp=%.2f, Ki=%.2f, Kd=%.2f", Kp_, Ki_, Kd_);
                rcl_interfaces::msg::SetParametersResult result;
                result.successful = true;
                return result;
            }
        );

        // Subscribe to velocity commands
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10,
            std::bind(&MotorControllerPID::cmdVelCallback, this, std::placeholders::_1)
        );

        // Setup publishers for debugging RPM
        target_rpm_pub_ = this->create_publisher<std_msgs::msg::Float64>("target_right_rpm", 10);
        actual_rpm_pub_ = this->create_publisher<std_msgs::msg::Float64>("actual_right_rpm", 10);

        RCLCPP_INFO(this->get_logger(), "MotorControllerPID node started");
    }

private:
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        // Fake simulation of actual RPM for demonstration
        double target_right_rpm = msg->linear.x * 60.0;  // Fake logic
        double actual_right_rpm = target_right_rpm - 5.0;  // Simulate slight error

        // Apply PID (dummy just for structure)
        double error = target_right_rpm - actual_right_rpm;
        integral_ += error;
        double derivative = error - prev_error_;
        double output = Kp_ * error + Ki_ * integral_ + Kd_ * derivative;
        prev_error_ = error;

        // Log and publish
        RCLCPP_INFO(this->get_logger(), "Target RPM: %.2f, Actual RPM: %.2f, Output: %.2f", target_right_rpm, actual_right_rpm, output);

        std_msgs::msg::Float64 msg_target;
        msg_target.data = target_right_rpm;
        target_rpm_pub_->publish(msg_target);

        std_msgs::msg::Float64 msg_actual;
        msg_actual.data = actual_right_rpm;
        actual_rpm_pub_->publish(msg_actual);
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr target_rpm_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr actual_rpm_pub_;

    // PID Variables
    double Kp_, Ki_, Kd_;
    double integral_ = 0.0;
    double prev_error_ = 0.0;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotorControllerPID>());
    rclcpp::shutdown();
    return 0;
}
