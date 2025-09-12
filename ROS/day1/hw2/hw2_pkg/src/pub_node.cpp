#include "hw2_pkg/pub.hpp"

MyPublisher::MyPublisher() : Node("publisher")
{
    string_pub_ = this->create_publisher<std_msgs::msg::String>("string_topic", 10);
    int_pub_ = this->create_publisher<std_msgs::msg::Int32>("int_topic", 10);
    float_pub_ = this->create_publisher<std_msgs::msg::Float32>("float_topic", 10);
}

void MyPublisher::publish_messages()
{
    auto str_msg = std_msgs::msg::String();
    string_pub_->publish(str_msg);
    RCLCPP_INFO(this->get_logger(), "String: %s", str_msg.data.c_str());
    
    auto int_msg = std_msgs::msg::Int32();
    int_pub_->publish(int_msg);
    RCLCPP_INFO(this->get_logger(), "Int: %d", int_msg.data);
    
    auto float_msg = std_msgs::msg::Float32();
    float_pub_->publish(float_msg);
    RCLCPP_INFO(this->get_logger(), "Float: %.1f", float_msg.data);

}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MyPublisher>();
    
    while (rclcpp::ok()) {
        node->publish_messages();
        rclcpp::spin_some(node);
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    
    rclcpp::shutdown();
    return 0;
}
