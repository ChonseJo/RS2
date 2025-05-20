#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <string>
#include <map>

// ⬇⬇⬇ added includes for capture-once API
#include <std_msgs/msg/empty.hpp>    // for trigger
#include <std_msgs/msg/string.hpp>   // for output
#include <sstream>
  
// Structure to hold detection information.
struct CapDetection {
    cv::Point2f centroid;
    float radius;
    std::string label;
    cv::Scalar color; // BGR
};

// Global function to merge detections that are close together.
std::vector<CapDetection> mergeDetections(const std::vector<CapDetection>& detections, float distanceThreshold) {
    std::vector<bool> merged(detections.size(), false);
    std::vector<CapDetection> mergedDetections;
    for (size_t i = 0; i < detections.size(); ++i) {
        if (merged[i]) continue;
        cv::Point2f sum = detections[i].centroid;
        float sumRadius = detections[i].radius;
        int count = 1;
        merged[i] = true;
        for (size_t j = i + 1; j < detections.size(); ++j) {
            if (merged[j]) continue;
            float dist = cv::norm(detections[i].centroid - detections[j].centroid);
            if (dist < distanceThreshold) {
                sum += detections[j].centroid;
                sumRadius += detections[j].radius;
                count++;
                merged[j] = true;
            }
        }
        cv::Point2f avg = sum * (1.0f / count);
        float avgRadius = sumRadius / count;
        mergedDetections.push_back({avg, avgRadius, detections[i].label, detections[i].color});
    }
    return mergedDetections;
}

class RealSenseCaptureNode : public rclcpp::Node {
public:
    RealSenseCaptureNode()
      : Node("realsense_capture_node")
    {
        // 1) Subscribe to the RealSense color image topic.
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/camera/color/image_raw", 10,
            std::bind(&RealSenseCaptureNode::image_callback, this, std::placeholders::_1)
        );

        // ⬇ added: publisher of the 6-slot sequence
        sequence_pub_ = this->create_publisher<std_msgs::msg::String>(
            "/bottle_sequence", 10
        );

        // ⬇ added: trigger subscriber to capture once
        capture_sub_ = this->create_subscription<std_msgs::msg::Empty>(
            "/capture_caps", 10,
            std::bind(&RealSenseCaptureNode::capture_callback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "RealSenseCaptureNode started, waiting for images...");
    }

private:
    // ⬇ added: buffer last frame
    cv::Mat last_frame_;

    // existing subscription
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;

    // ⬇ added: pub/sub for capture API
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr sequence_pub_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr  capture_sub_;

    // 1) image_callback now also buffers
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
        } catch (cv_bridge::Exception &e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }
        last_frame_ = cv_ptr->image.clone();          // ⬅ added
        processImage(cv_ptr->image);
        if (cv::waitKey(1) == 27) {
            rclcpp::shutdown();
        }
    }

    // 2) capture_callback: compute & publish once
    void capture_callback(const std_msgs::msg::Empty::SharedPtr) {
        if (last_frame_.empty()) {
            RCLCPP_WARN(get_logger(), "No frame available to capture.");
            return;
        }
        auto seq = computeGridSequence(last_frame_);  // ⬅ added
        std_msgs::msg::String out;
        out.data = seq;
        sequence_pub_->publish(out);
        RCLCPP_INFO(get_logger(), "Published bottle sequence: '%s'", seq.c_str());
    }

    // your existing processImage unchanged:
    void processImage(const cv::Mat &image) {
        // 1. Convert to HSV.
        cv::Mat hsv;
        cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);

        // 2. Define masks for red (double range), green, and blue.
        cv::Mat r1, r2, red_mask, green_mask, blue_mask;
        cv::inRange(hsv, cv::Scalar(  0, 110, 80), cv::Scalar( 10, 255, 255), r1);
        cv::inRange(hsv, cv::Scalar(170, 110, 80), cv::Scalar(180, 255, 255), r2);
        red_mask = r1 | r2;

        cv::inRange(hsv, cv::Scalar(35, 50, 50), cv::Scalar(85, 255, 255), green_mask);
        cv::inRange(hsv, cv::Scalar(105, 150, 50), cv::Scalar(125, 255, 255), blue_mask);

        // 3. Apply morphological operations.
        cv::Mat k5 = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5,5));
        cv::Mat k9 = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(9,9));
        auto clean = [&](cv::Mat &m, const cv::Mat &kernel, int iter){
            cv::morphologyEx(m, m, cv::MORPH_CLOSE, kernel, cv::Point(-1,-1), iter);
            cv::morphologyEx(m, m, cv::MORPH_OPEN,  kernel, cv::Point(-1,-1), iter);
        };
        clean(red_mask,   k5, 2);
        clean(blue_mask,  k5, 2);
        clean(green_mask, k9, 3);

        // 4. Find contours & compute centroids.
        auto find_centroids = [&](const cv::Mat& mask,
                                  const std::string& label,
                                  cv::Scalar color,
                                  bool checkCircularity = false)
        {
            std::vector<std::vector<cv::Point>> contours;
            cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
            std::vector<CapDetection> detections;
            for (auto &cnt : contours) {
                double area = cv::contourArea(cnt);
                if (area < 2000) continue;
                cv::Moments M = cv::moments(cnt);
                if (M.m00 == 0) continue;
                if (checkCircularity) {
                    double perim = cv::arcLength(cnt, true);
                    double circ = 4 * CV_PI * area / (perim * perim);
                    if (circ < 0.7) continue;
                }
                cv::Point2f c(M.m10 / M.m00, M.m01 / M.m00);
                float radius;
                cv::minEnclosingCircle(cnt, c, radius);
                detections.push_back({c, radius, label, color});
            }
            return detections;
        };

        std::vector<CapDetection> allDetections;
        auto redDet   = find_centroids(red_mask,   "Coke",   cv::Scalar(0,0,255), true);
        auto greenDet = find_centroids(green_mask, "Sprite", cv::Scalar(0,255,0), false);
        auto blueDet  = find_centroids(blue_mask,  "Fanta",  cv::Scalar(255,0,0), false);
        allDetections.insert(allDetections.end(), redDet.begin(),   redDet.end());
        allDetections.insert(allDetections.end(), greenDet.begin(), greenDet.end());
        allDetections.insert(allDetections.end(), blueDet.begin(),  blueDet.end());

        auto mergedDetections = mergeDetections(allDetections, 50.0f);

        cv::Mat dotImage = cv::Mat::zeros(image.size(), image.type());
        for (auto &d : mergedDetections) {
            cv::circle(dotImage, d.centroid, (int)d.radius, d.color, 3);
            cv::putText(dotImage, d.label,
                        d.centroid + cv::Point2f(-d.radius*0.5f, d.radius+20),
                        cv::FONT_HERSHEY_SIMPLEX, 0.7, d.color, 2);
        }

        cv::imshow("Camera View",   image);
        cv::imshow("Bottle Type", dotImage);
    }

    // ⬇ added: compute the 2×3‐grid sequence
    std::string computeGridSequence(const cv::Mat &image) {
        cv::Mat hsv; 
        cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);
        cv::Mat r1, r2, red_mask, green_mask, blue_mask;
        cv::inRange(hsv, cv::Scalar(0,200,100), cv::Scalar(10,255,180), r1);
        cv::inRange(hsv, cv::Scalar(170,200,50), cv::Scalar(180,255,180), r2);
        red_mask = r1 | r2;
        cv::inRange(hsv, cv::Scalar(35,50,50),  cv::Scalar(85,255,255), green_mask);
        cv::inRange(hsv, cv::Scalar(105,150,50),cv::Scalar(125,255,255), blue_mask);
        auto k5 = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5,5));
        auto k9 = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(9,9));
        auto clean = [&](cv::Mat &m, const cv::Mat &k, int it){
            cv::morphologyEx(m,m,cv::MORPH_CLOSE,k,cv::Point(-1,-1),it);
            cv::morphologyEx(m,m,cv::MORPH_OPEN, k,cv::Point(-1,-1),it);
        };
        clean(red_mask,   k5,2);
        clean(blue_mask,  k5,2);
        clean(green_mask, k9,3);

        int cellW = image.cols/3, cellH = image.rows/2;
        std::vector<char> codes(6,'?');
        std::map<int,char> cmap = {{1,'C'},{2,'S'},{3,'F'}};
        for (int r=0; r<2; ++r) {
            for (int c=0; c<3; ++c) {
                cv::Rect roi(c*cellW, r*cellH, cellW, cellH);
                int cr = cv::countNonZero(red_mask  (roi));
                int cg = cv::countNonZero(green_mask(roi));
                int cb = cv::countNonZero(blue_mask (roi));
                int best = (cr>cg&&cr>cb)?1:(cg>cr&&cg>cb)?2:(cb>cr&&cb>cg)?3:0;
                codes[r*3 + c] = best ? cmap[best] : '?';
            }
        }
        std::ostringstream oss;
        for (int i=0; i<6; ++i) {
            oss << codes[i] << (i==2 ? " | " : (i<5 ? " " : ""));
        }
        return oss.str();
    }
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RealSenseCaptureNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
