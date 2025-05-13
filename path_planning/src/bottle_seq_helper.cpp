#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class BottleSequenceHelper : public rclcpp::Node
{
  public:
    ServoControl() : Node("servo_control")
    {
        publisher_ = this->create_publisher<std_msgs::msg::String>("/bottle_sequence", 10);
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "/bottle_sequence", 10,
            std::bind(&BottleSequenceHelper::topic_callback, this, std::placeholders::_1));
    
        RCLCPP_INFO(this->get_logger(), "BottleSequenceHelper node started");
    }

    std::vector<size_t> get_bottle_positions(const std::string &type) const
    {
        std::vector<size_t> positions;
        for (size_t i = 0; i < bottle_sequence_.size(); ++i) {
            if (bottle_sequence_[i] == type) {
                positions.push_back(i);
            }
        }
        return positions;
    }

  private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};
