#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"

class MySubscriberNode : public rclcpp::Node
{
public:
    MySubscriberNode();

private:
    void int_callback(const std_msgs::msg::Int32::SharedPtr msg);
    void string_callback(const std_msgs::msg::String::SharedPtr msg);
    void float_callback(const std_msgs::msg::Float32::SharedPtr msg);

    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_int_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_str_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_float_;
};