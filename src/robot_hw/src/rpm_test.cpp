#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/twist.hpp"

using std::placeholders::_1;

class RpmTest : public rclcpp::Node {
public:
    RpmTest()
        : Node("motor_controller_pid"), prev_time_(this->now()) {
        
        ms_to_rpm_ = (1/(wheel_radius_ * 2 * 3.1415)) * 60;

        ticks_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(
            "ticks", 10, std::bind(&RpmTest::ticks_callback, this, _1));
    }

private:

    void ticks_callback(const geometry_msgs::msg::Vector3::SharedPtr msg) {
        rclcpp::Time now = this->now();
        double dt = (now - prev_time_).seconds();
        prev_time_ = now;
        
        if (dt == 0) return;

        double d_left_ticks = msg->x - prev_left_ticks;
        double d_right_ticks = msg->y - prev_right_ticks;

        prev_left_ticks = msg->x;
        prev_right_ticks = msg->y;

        // Convert ticks to RPM
        double actual_left_rpm = (d_left_ticks / dt) * 60.0 / encoder_ticks_per_rev_;
        double actual_right_rpm = (d_right_ticks / dt) * 60.0 / encoder_ticks_per_rev_;

        RCLCPP_INFO(this->get_logger(), "left_rpm: : %f", actual_left_rpm);
        RCLCPP_INFO(this->get_logger(), "right_rpm: : %f", actual_right_rpm);

        RCLCPP_INFO(this->get_logger(), "----------------------------------------------------");

    }

    
    double ms_to_rpm_;
    double encoder_ticks_per_rev_ = 160.0;
    double wheel_radius_ = 0.065;
    
    double prev_left_ticks = 0.0, prev_right_ticks = 0.0;
    
    rclcpp::Time prev_time_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr ticks_sub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RpmTest>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}