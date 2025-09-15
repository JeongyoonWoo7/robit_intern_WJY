#include "rclcpp/rclcpp.hpp"
#include "custom_interfaces/msg/add_two_ints.hpp"

using std::placeholders::_1;

class VectorPublisher : public rclcpp::Node
{
public:
    VectorPublisher() : Node("vector_publisher"), count_(0)
    {
        pub_ = this->create_publisher<custom_interfaces::msg::AddTwoInts>("vector_topic", 10);
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&VectorPublisher::timer_callback, this));
    }

private:
    void timer_callback()
    {
        auto msg = custom_interfaces::msg::AddTwoInts();
        msg.a = count_;  
        msg.b = {count_, count_+1, count_+2};  

        RCLCPP_INFO(this->get_logger(), "Publishing: a=%ld b=[%d, %d, %d]",
                    msg.a, msg.b[0], msg.b[1], msg.b[2]);

        pub_->publish(msg);
        count_++;
    }

    rclcpp::Publisher<custom_interfaces::msg::AddTwoInts>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    int count_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VectorPublisher>());
    rclcpp::shutdown();
    return 0;
}
