#include "rclcpp/rclcpp.hpp"
#include "custom_interfaces/msg/add_two_ints.hpp"

class VectorSubscriber : public rclcpp::Node
{
public:
    VectorSubscriber() : Node("vector_subscriber")
    {
        sub_ = this->create_subscription<custom_interfaces::msg::AddTwoInts>(
            "vector_topic", 10,
            std::bind(&VectorSubscriber::topic_callback, this, std::placeholders::_1));
    }

private:
    void topic_callback(const custom_interfaces::msg::AddTwoInts::SharedPtr msg)
    {
        std::ostringstream oss;
        oss << "a=" << msg->a << " b=[";
        for (size_t i = 0; i < msg->b.size(); i++) {
            oss << msg->b[i];
            if (i + 1 < msg->b.size()) oss << ", ";
        }
        oss << "]";
        RCLCPP_INFO(this->get_logger(), "I heard: %s", oss.str().c_str());
    }

    rclcpp::Subscription<custom_interfaces::msg::AddTwoInts>::SharedPtr sub_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VectorSubscriber>());
    rclcpp::shutdown();
    return 0;
}
