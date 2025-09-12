#ifndef SUB_HPP_
#define SUB_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/float32.hpp"

class MySubscriber : public rclcpp::Node
{
public:
    MySubscriber();

private:
    void string_callback(const std_msgs::msg::String::SharedPtr msg);
    void int_callback(const std_msgs::msg::Int32::SharedPtr msg);
    void float_callback(const std_msgs::msg::Float32::SharedPtr msg);
    
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr string_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr int_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr float_sub_;
};

#endif
