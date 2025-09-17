#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

class JoySim : public rclcpp::Node
{
public:
    JoySim()
    : Node("joysim"), x_(0.0), y_(0.0), theta_(0.0)
    {
        
        sub_cmd_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10, std::bind(&JoySim::cmd_callback, this, std::placeholders::_1));

        
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

        last_time_ = this->now();

        RCLCPP_INFO(this->get_logger(), "joysim node started, listening to /cmd_vel");
    }

private:
    void cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        rclcpp::Time current_time = this->now();
        double dt = (current_time - last_time_).seconds();
        last_time_ = current_time;

        
        double vx = msg->linear.x;
        double vth = msg->angular.z;

       
        x_ += vx * cos(theta_) * dt;
        y_ += vx * sin(theta_) * dt;
        theta_ += vth * dt;

        
        tf2::Quaternion q;
        q.setRPY(0, 0, theta_);

        
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = current_time;
        t.header.frame_id = "map";
        t.child_frame_id = "odom";
        t.transform.translation.x = x_;
        t.transform.translation.y = y_;
        t.transform.translation.z = 0.0;
        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();

       
        tf_broadcaster_->sendTransform(t);

        RCLCPP_INFO(this->get_logger(), "Odom updated: x=%.2f, y=%.2f, theta=%.2f",
                    x_, y_, theta_);
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    double x_, y_, theta_;
    rclcpp::Time last_time_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JoySim>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

