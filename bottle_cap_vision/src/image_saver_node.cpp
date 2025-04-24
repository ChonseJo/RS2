#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class ImageSaverNode : public rclcpp::Node {
public:
  ImageSaverNode() : Node("image_saver_node"), captured_(false) {
    // Subscribe to the RealSense color image topic.
    // Change the topic if needed (e.g., "/camera/camera/color/image_raw")
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "camera/camera/color/image_raw", 10,
      std::bind(&ImageSaverNode::image_callback, this, std::placeholders::_1)
    );
    RCLCPP_INFO(this->get_logger(), "ImageSaverNode started. Waiting for image...");
  }

private:
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    if (captured_) {
      return; // Already captured one frame.
    }

    RCLCPP_INFO(this->get_logger(), "Received an image frame!");
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
    } catch (cv_bridge::Exception &e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    // Save the image to disk.
    std::string filename = "captured_image.jpg";
    if (cv::imwrite(filename, cv_ptr->image)) {
      RCLCPP_INFO(this->get_logger(), "Saved image to %s", filename.c_str());
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to save image.");
    }
    
    captured_ = true;
    // Optionally, shutdown after saving the image.
    rclcpp::shutdown();
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  bool captured_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ImageSaverNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
