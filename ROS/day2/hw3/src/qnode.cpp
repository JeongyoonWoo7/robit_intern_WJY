#include "communicator/qnode.hpp"

QNode::QNode() : QObject(), rclcpp::Node("qt_communicator")
{
    publisher_ = this->create_publisher<std_msgs::msg::String>("chatter", 10);

    subscriber_ = this->create_subscription<std_msgs::msg::String>(
        "chatter", 10,
        [this](std_msgs::msg::String::SharedPtr msg) {
            QString qmsg = QString::fromStdString(msg->data);
            emit messageReceived(qmsg);
        });
}

void QNode::publishMessage(const std::string &msg)
{
    auto message = std_msgs::msg::String();
    message.data = msg;
    publisher_->publish(message);
}

