#include "hw2_pkg/sub.hpp"

MySubscriber::MySubscriber() : Node("subscriber")
{
    string_sub_ = this->create_subscription<std_msgs::msg::String>(
        "string_topic", 10,
        std::bind(&MySubscriber::string_callback, this, std::placeholders::_1)
    );
    
    int_sub_ = this->create_subscription<std_msgs::msg::Int32>(
        "int_topic", 10,
        std::bind(&MySubscriber::int_callback, this, std::placeholders::_1)
    );
    
    float_sub_ = this->create_subscription<std_msgs::msg::Float32>(
        "float_topic", 10,
        std::bind(&MySubscriber::float_callback, this, std::placeholders::_1)
    );
    
    RCLCPP_INFO(this->get_logger(), "Subscriber 시작");
}

void MySubscriber::string_callback(const std_msgs::msg::String::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "받은 문자열: %s", msg->data.c_str());
}

void MySubscriber::int_callback(const std_msgs::msg::Int32::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "받은 정수: %d", msg->data);
}

void MySubscriber::float_callback(const std_msgs::msg::Float32::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "받은 실수: %.1f", msg->data);
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MySubscriber>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
