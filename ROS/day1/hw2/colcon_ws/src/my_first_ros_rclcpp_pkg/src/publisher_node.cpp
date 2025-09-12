#include "my_first_ros_rclcpp_pkg/publisher_node.hpp"


MyPublisherNode::MyPublisherNode() : Node("publisher_node"), count_(0)
{
    pub_int_ = this->create_publisher<std_msgs::msg::Int32>("topic_int", 10);
    pub_str_ = this->create_publisher<std_msgs::msg::String>("topic_str", 10);
    pub_float_ = this->create_publisher<std_msgs::msg::Float32>("topic_float", 10);

    timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&MyPublisherNode::timer_callback, this));
}


void MyPublisherNode::timer_callback()
{
    auto msg_int = std_msgs::msg::Int32();
    msg_int.data = count_;
    pub_int_->publish(msg_int);

    auto msg_str = std_msgs::msg::String();
    msg_str.data = "Hello from c++" + std::to_string(count_);
    pub_str_->publish(msg_str);

    auto msg_float = std_msgs::msg::Float32();
    msg_float.data = 0.1f * count_;
    pub_float_->publish(msg_float);

    RCLCPP_INFO(this->get_logger(), "Publishing: int=%d, str=%s, float=%.2f",
                msg_int.data, msg_str.data.c_str(), msg_float.data);

    count_++;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MyPublisherNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}




