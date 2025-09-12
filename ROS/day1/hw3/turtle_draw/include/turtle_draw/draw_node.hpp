#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/srv/set_pen.hpp"

class TurtleDraw : public rclcpp::Node
{
public:
    TurtleDraw();

private:
    
    void timer_callback();

    

    void drawCircle();
    void drawTriangle();
    void drawSquare();
    void stop();



   
    
    void setPenColor(uint8_t r, uint8_t g, uint8_t b, uint8_t width, bool off);

    
    
    void keyLoop();

   
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Client<turtlesim::srv::SetPen>::SharedPtr pen_client_;

    
    int step_;
    int shape_mode_; 
    rclcpp::Time last_change_time_;
};