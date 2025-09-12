#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"

class MyPublisherNode : public rclcpp::Node
{
public:
    MyPublisherNode();

private:
    void timer_callback();

    int count_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_int_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_str_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_float_;
};
