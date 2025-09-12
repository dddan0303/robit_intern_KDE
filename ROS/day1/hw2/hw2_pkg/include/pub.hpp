#ifndef PUB_HPP_
#define PUB_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/float32.hpp"

class MyPublisher : public rclcpp::Node
{
public:
    MyPublisher();
    void publish_messages();

private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr string_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr int_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr float_pub_;
    
    int count_;
};

#endif
