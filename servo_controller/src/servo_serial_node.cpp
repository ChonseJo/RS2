#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <serial/serial.h>

class ServoSerialNode : public rclcpp::Node {
public:
  ServoSerialNode() : Node("servo_serial_node") {
    try {
      serial_port_.setPort("/dev/ttyUSB0");
      serial_port_.setBaudrate(9600);
      serial::Timeout to = serial::Timeout::simpleTimeout(1000);
      serial_port_.setTimeout(to);
      serial_port_.open();
    } catch (serial::IOException &e) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open serial port.");
    }

    if (serial_port_.isOpen()) {
      RCLCPP_INFO(this->get_logger(), "Serial port open.");
    }

    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "servo_command", 10,
      std::bind(&ServoSerialNode::command_callback, this, std::placeholders::_1)
    );
  }

private:
  void command_callback(const std_msgs::msg::String::SharedPtr msg) {
    std::string command = msg->data + "\n";  // Add newline for Arduino
    serial_port_.write(command);
    RCLCPP_INFO(this->get_logger(), "Sent command: %s", msg->data.c_str());
  }

  serial::Serial serial_port_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ServoSerialNode>());
  rclcpp::shutdown();
  return 0;
}
