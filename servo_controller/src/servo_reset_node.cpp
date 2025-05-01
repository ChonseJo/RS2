#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class ServoResetNode : public rclcpp::Node {
public:
  ServoResetNode() : Node("servo_reset_node") {
    publisher_ = this->create_publisher<std_msgs::msg::String>("servo_command", 10);

    auto msg = std_msgs::msg::String();
    msg.data = "SET:95,95,95,95,95,95";

    publisher_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Published reset command: %s", msg.data.c_str());

    // Shut down immediately after publishing
    rclcpp::shutdown();
  }

private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  std::make_shared<ServoResetNode>();  // node publishes on construction
  rclcpp::spin_some(rclcpp::Node::make_shared("noop"));  // allow publish to complete
  return 0;
}
