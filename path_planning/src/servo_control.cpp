#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class ServoControl : public rclcpp::Node
{
  public:
    ServoControl() : Node("servo_control")
    {
      publisher_ = this->create_publisher<std_msgs::msg::String>(
        "servo_command", 10
      );
    }

    void close_servos()
    {
        auto msg = std_msgs::msg::String();
        msg.data = "SET:100,100,100,100,100,100";
        publisher_->publish(msg);

        RCLCPP_INFO(this->get_logger(), "Closing All Servos...");
    }

    void open_servos(const std::vector<int>& servoState)
    {
        std::string str = "SET:";
        for (size_t i = 0; i < servoState.size(); ++i) {
            if (servoState[i] == 0) {
                str += "100";
            } else if (servoState[i] == 1) {
                str += "160";
            }
            if (i + 1 != servoState.size()) {
                str += ",";
            }
        }
        // str += "\n";
        auto msg = std_msgs::msg::String();
        msg.data = str;
        publisher_->publish(msg);

        RCLCPP_INFO(this->get_logger(), "Opening Servos, %s", msg.data.c_str());
    }

  private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  };
