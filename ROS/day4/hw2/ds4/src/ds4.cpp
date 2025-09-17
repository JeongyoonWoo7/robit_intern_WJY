#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"

class DS4Teleop : public rclcpp::Node
{
public:
    DS4Teleop()
    : Node("ds4")
    {
        
        pub_cmd_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
       
        sub_joy_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10, std::bind(&DS4Teleop::joy_callback, this, std::placeholders::_1));

        max_lin_ = 1.0; // m/s
        max_ang_ = 1.0; // rad/s
    }

private:
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        auto twist = geometry_msgs::msg::Twist();

       
        twist.linear.x = max_lin_ * msg->axes[1];
        twist.angular.z = max_ang_ * msg->axes[0];

        pub_cmd_->publish(twist);

        RCLCPP_INFO(this->get_logger(), "linear.x=%.2f, angular.z=%.2f",
                    twist.linear.x, twist.angular.z);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_joy_;

    double max_lin_;
    double max_ang_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DS4Teleop>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

