#include <iostream>
<<<<<<< HEAD
#include <vector>
#include <string>
#include <array>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/empty.hpp"
=======
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
>>>>>>> b27819e3d911ec6cec2d98839386003e0e73d2ca

class ServoControl : public rclcpp::Node
{
  public:
    ServoControl() : Node("servo_control")
    {
      publisher_ = this->create_publisher<std_msgs::msg::String>(
<<<<<<< HEAD
        "servo_command", 10);

      cam_publisher_ = this->create_publisher<std_msgs::msg::Empty>("/capture_caps", 10);
      cam_subscription_ = this->create_subscription<std_msgs::msg::String>(
          "/bottle_sequence", 10,
          std::bind(&ServoControl::topic_callback, this, std::placeholders::_1));
=======
        "servo_command", 10
      );
>>>>>>> b27819e3d911ec6cec2d98839386003e0e73d2ca
    }

    void close_servos()
    {
        auto msg = std_msgs::msg::String();
<<<<<<< HEAD
        msg.data = "SET:" + closed + "," + closed + "," + closed + "," + closed + ","
                    + closed + "," + closed;
=======
        msg.data = "SET:160,160,160,160,160,160";
>>>>>>> b27819e3d911ec6cec2d98839386003e0e73d2ca
        publisher_->publish(msg);

        RCLCPP_INFO(this->get_logger(), "Closing All Servos...");
    }

    void open_servos(const std::vector<int>& servoState)
    {
        std::string str = "SET:";
        for (size_t i = 0; i < servoState.size(); ++i) {
            if (servoState[i] == 0) {
<<<<<<< HEAD
                str += closed;
            } else if (servoState[i] == 1) {
                str += open;
=======
                str += "160";
            } else if (servoState[i] == 1) {
                str += "100";
>>>>>>> b27819e3d911ec6cec2d98839386003e0e73d2ca
            }
            if (i + 1 != servoState.size()) {
                str += ",";
            }
        }
<<<<<<< HEAD
=======
        // str += "\n";
>>>>>>> b27819e3d911ec6cec2d98839386003e0e73d2ca
        auto msg = std_msgs::msg::String();
        msg.data = str;
        publisher_->publish(msg);

        RCLCPP_INFO(this->get_logger(), "Opening Servos, %s", msg.data.c_str());
    }

<<<<<<< HEAD
    void openBottleTypes(const std::string &type)
    {
        auto state = getBottlePositions(type);
        open_servos(state);
    }

    std::vector<bool>  requestBottlePositions(){
        std_msgs::msg::Empty msg;
        cam_publisher_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Requested bottle positions via cam trigger.");

        while(!sequence_received_){ /* do nothing */ }

        std::vector<bool> bottles;
        bottles.push_back(isBottleType("C"));
        bottles.push_back(isBottleType("F"));
        bottles.push_back(isBottleType("S"));

        sequence_received_ = false;
        return bottles;
    }



  private:

    std::vector<int> getBottlePositions(const std::string &type) const
    {
        std::vector<int> state;
        for (size_t i = 0; i < bottle_sequence_.size(); i++) {
            state.push_back(bottle_sequence_[i] == type ? 1 : 0);
        }
        return state;
    }

    bool isBottleType(const std::string &type)
    {
        for (size_t i = 0; i < bottle_sequence_.size(); i++) {
            if (bottle_sequence_[i] == type){
                return true;
            }
        }
        return false;
    }

    void topic_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received sequence: '%s'", msg->data.c_str());
        parse_sequence(msg->data);
        sequence_received_ = true;
    }

    void parse_sequence(const std::string &sequence)
    {
        std::istringstream ss(sequence);
        std::string token;
        size_t idx = 0;
    
        while (ss >> token && idx < bottle_sequence_.size()) {
            if (token == "|") {
                continue;  // Skip separator
            }
            bottle_sequence_[idx++] = token;
        }
    
        // Fill remaining entries with "?" if the input was short
        for (; idx < bottle_sequence_.size(); ++idx) {
            bottle_sequence_[idx] = "?";
        }
    
        RCLCPP_INFO(this->get_logger(), "Parsed sequence: ");
        for (const auto &b : bottle_sequence_) {
            std::cout << b << " ";
        }
        std::cout << std::endl;
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr cam_publisher_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr cam_subscription_;
    std::array<std::string, 6> bottle_sequence_;
    bool sequence_received_ = false;

    std::string open = "100";
    std::string closed = "140";
=======
  private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
>>>>>>> b27819e3d911ec6cec2d98839386003e0e73d2ca
  };
