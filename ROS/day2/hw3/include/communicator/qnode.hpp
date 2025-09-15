#pragma once
#include <QObject>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class QNode : public QObject, public rclcpp::Node
{
    Q_OBJECT
public:
    QNode();
    void publishMessage(const std::string &msg);

signals:
    void messageReceived(QString msg);

private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_;
};

