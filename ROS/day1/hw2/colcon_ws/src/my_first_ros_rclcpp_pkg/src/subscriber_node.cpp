#include "my_first_ros_rclcpp_pkg/subscriber_node.hpp"

MySubscriberNode::MySubscriberNode() : Node("cpp_subscriber")
{
    sub_int_ = this->create_subscription<std_msgs::msg::Int32>(
        "int_topic", 10,
        std::bind(&MySubscriberNode::int_callback, this, std::placeholders::_1));

    sub_str_ = this->create_subscription<std_msgs::msg::String>(
        "string_topic", 10,
        std::bind(&MySubscriberNode::string_callback, this, std::placeholders::_1));

    sub_float_ = this->create_subscription<std_msgs::msg::Float32>(
        "float_topic", 10,
        std::bind(&MySubscriberNode::float_callback, this, std::placeholders::_1));
}

void MySubscriberNode::int_callback(const std_msgs::msg::Int32::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Received Int: %d", msg->data);
}

void MySubscriberNode::string_callback(const std_msgs::msg::String::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Received String: %s", msg->data.c_str());
}

void MySubscriberNode::float_callback(const std_msgs::msg::Float32::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Received Float: %.2f", msg->data);
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MySubscriberNode>());
    rclcpp::shutdown();
    return 0;
}
