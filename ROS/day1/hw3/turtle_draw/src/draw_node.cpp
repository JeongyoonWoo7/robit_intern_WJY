#include "turtle_draw/draw_node.hpp"
#include "geometry_msgs/msg/twist.hpp"



class TurtleDraw : public rclcpp :: node
{
    public:
       TurtleDraw() : Node("turtle_draw")
       {
            pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);

            timer_ this_>create_wall_timer(
                std::chrono::miliseconds(100),
                std::bind(%TurtleDraw::timer_callback, this));
            }

        

    private:
            void timer_callback()
            {
                geometry_msgs::msg::Twist msg;
                msg.linear.x = 2.0;
                msg.angular.z = 1.8;
                pub_ -> publish(msg);
            }

            rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
            rclcpp::TimerBase::SharedPtr timer_;
        
};



void TurtleDraw::drawCircle()
{
    geometry_msgs::msg::Twist msg;
    msg.linear.x = 2.0;
    msg.angular.z = 1.8;
    pub_->publish(msg);
}

void TurtleDraw::drawTriangle()
{
    geometry_msgs::msg::Twist msg;
    auto now = this->now();

    if ((now - last_change_time_).seconds() < 3.0) {
        msg.linear.x = 2.0;
    } else if ((now - last_change_time_).seconds() < 4.0) {
        msg.angular.z = 2.1;
    } else {
        last_change_time_ = now;
        step_++;
        if (step_ >= 3) { stop(); shape_mode_ = -1; step_ = 0; }
    }
    pub_->publish(msg);
}

void TurtleDraw::drawSquare()
{
    geometry_msgs::msg::Twist msg;
    auto now = this->now();

    if ((now - last_change_time_).seconds() < 3.0) {
        msg.linear.x = 2.0;
    } else if ((now - last_change_time_).seconds() < 4.5) {
        msg.angular.z = 1.57;
    } else {
        last_change_time_ = now;
        step_++;
        if (step_ >= 4) { stop(); shape_mode_ = -1; step_ = 0; }
    }
    pub_->publish(msg);
}



int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<draw_node();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}



