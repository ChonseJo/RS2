#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class ServoTestNode : public rclcpp::Node {
public:
  ServoTestNode() : Node("servo_test_node") {
    pub_ = this->create_publisher<std_msgs::msg::String>("servo_command", 10);
    timer_ = this->create_wall_timer(
      std::chrono::seconds(3),
      std::bind(&ServoTestNode::toggle_servos, this)
    );
  }

private:
  void toggle_servos() {
    std_msgs::msg::String msg;
    msg.data = is_open_ ? "SET:160,160,160,160,160,160" : "SET:95,95,95,95,95,95";
    pub_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Sent command: %s", msg.data.c_str());
    is_open_ = !is_open_;
  }

  bool is_open_ = false;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ServoTestNode>());
  rclcpp::shutdown();
  return 0;
}
