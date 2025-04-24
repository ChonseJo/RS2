#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <string>

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
        // Subscribe to the RealSense color image topic.
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/camera/color/image_raw", 10,
            std::bind(&RealSenseCaptureNode::image_callback, this, std::placeholders::_1)
        );

        // Create display windows.
        // cv::namedWindow("Captured Image", cv::WINDOW_AUTOSIZE);
        // cv::namedWindow("Dot Representation", cv::WINDOW_AUTOSIZE);

        // h_lo_ = 0;   h_hi_ = 10;
        // s_lo_ = 50;  s_hi_ = 255;
        // v_lo_ = 50;  v_hi_ = 255;
        // cv::namedWindow("tuners", cv::WINDOW_AUTOSIZE);
        // cv::createTrackbar("h_lo","tuners",&h_lo_,180);
        // cv::createTrackbar("h_hi","tuners",&h_hi_,180);
        // cv::createTrackbar("s_lo","tuners",&s_lo_,255);
        // cv::createTrackbar("s_hi","tuners",&s_hi_,255);
        // cv::createTrackbar("v_lo","tuners",&v_lo_,255);
        // cv::createTrackbar("v_hi","tuners",&v_hi_,255);

        RCLCPP_INFO(this->get_logger(), "RealSenseCaptureNode started, waiting for images...");
    }

private:
    // int h_lo_, h_hi_, s_lo_, s_hi_, v_lo_, v_hi_;

    // rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
        } catch (cv_bridge::Exception &e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        processImage(cv_ptr->image);
        int key = cv::waitKey(1);
        if (key == 27) {  // ESC key
            rclcpp::shutdown();
        }
    }

    void processImage(const cv::Mat &image) {
        // 1. Convert to HSV.
        cv::Mat hsv;
        cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);

        // cv::Mat tune_mask;
        // cv::inRange(hsv,
        //             cv::Scalar(h_lo_, s_lo_, v_lo_),
        //             cv::Scalar(h_hi_, s_hi_, v_hi_),
        //             tune_mask);
        // cv::imshow("tuners", tune_mask);

        cv::Mat r1, r2;
        cv::inRange(hsv, cv::Scalar(0,200,100), cv::Scalar(10, 255, 180), r1);
        cv::inRange(hsv, cv::Scalar(170,200,50), cv::Scalar(180,255,180), r2);
        cv::Mat red_mask = r1 | r2;


        cv::Mat green_mask;
        cv::inRange(hsv, cv::Scalar(35, 50, 50), cv::Scalar(85, 255, 255), green_mask);

        cv::Mat blue_mask;
        cv::inRange(hsv, cv::Scalar(105, 150, 50), cv::Scalar(125, 255, 255), blue_mask);

        // Debug view
        // cv::imshow("Red Mask",   red_mask);
        // cv::imshow("Green Mask", green_mask);
        // cv::imshow("Blue Mask",  blue_mask);


        // // 3. Apply morphological operations.
        cv::medianBlur(red_mask,   red_mask,   7);
        cv::medianBlur(green_mask, green_mask, 7);
        cv::medianBlur(blue_mask,  blue_mask,  7);

        // 2) Use a slightly bigger elliptical kernel for green
        auto k5  = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5,5));
        auto k9  = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(9,9));

        // 3) Close then open on all three — but give green a stronger pass
        auto clean = [&](cv::Mat &m, const cv::Mat &kernel, int iter){
            cv::morphologyEx(m, m, cv::MORPH_CLOSE, kernel, cv::Point(-1,-1), iter);
            cv::morphologyEx(m, m, cv::MORPH_OPEN,  kernel, cv::Point(-1,-1), iter);
        };

        clean(red_mask,   k5, 2);   // small kernel, light clean
        clean(blue_mask,  k5, 2);   // small kernel, light clean
        clean(green_mask, k9, 3);   // bigger kernel, heavier clean

        // ##################################################
        const int cellW = image.cols / 3;
        const int cellH = image.rows / 2;

        // ROI cell lookup helper, now in the right scope:
        auto findCellIndex = [&](const cv::Point2f &pt) {
            int col = std::min(int(pt.x / cellW), 2);  // 0..2
            int row = std::min(int(pt.y / cellH), 1);  // 0..1
            return row * 3 + col;                      // 0..5
        };

        // count which colour dominates each of the 6 slots
        std::vector<int> slotColours;
        slotColours.reserve(6);
        for (int row = 0; row < 2; ++row) {
            for (int col = 0; col < 3; ++col) {
                cv::Rect roi(
                  col * cellW,
                  row * cellH,
                  cellW, cellH
                );
                int cr = cv::countNonZero(red_mask  (roi));
                int cg = cv::countNonZero(green_mask(roi));
                int cb = cv::countNonZero(blue_mask (roi));
                if      (cr > cg && cr > cb) slotColours.push_back(1);
                else if (cg > cr && cg > cb) slotColours.push_back(2);
                else if (cb > cr && cb > cg) slotColours.push_back(3);
                else                          slotColours.push_back(0);
            }
        }
        RCLCPP_INFO(
          this->get_logger(),
          "Slots R→B: [%d %d %d | %d %d %d]",
          slotColours[0], slotColours[1], slotColours[2],
          slotColours[3], slotColours[4], slotColours[5]
        );


        // 4. Find contours & compute centroids (with optional circularity).
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

        // Gather all detections.
        std::vector<CapDetection> allDetections;
        // red: enforce circular shape
        auto redDet   = find_centroids(red_mask,   "Coke",   cv::Scalar(0,0,255), true);
        // green: no circularity check
        auto greenDet = find_centroids(green_mask, "Sprite", cv::Scalar(0,255,0), false);
        // blue: no circularity check
        auto blueDet  = find_centroids(blue_mask,  "Fanta",  cv::Scalar(255,0,0), false);

        allDetections.insert(allDetections.end(), redDet.begin(),   redDet.end());
        allDetections.insert(allDetections.end(), greenDet.begin(), greenDet.end());
        allDetections.insert(allDetections.end(), blueDet.begin(),  blueDet.end());

        // 5. Merge nearby detections into one.
        float mergeThreshold = 50.0f;
        auto mergedDetections = mergeDetections(allDetections, mergeThreshold);

        std::vector<char> gridCodes(6, '?');
        // map labels to single‑char codes
        static const std::map<std::string,char> codeMap = {
            {"Coke",   'C'},
            {"Sprite", 'S'},
            {"Fanta",  'F'}
        };
        
        for (auto &d : mergedDetections) {
            int idx = findCellIndex(d.centroid);  // 0..5
            gridCodes[idx] = codeMap.at(d.label);
        }
        
        // print your 6‑letter layout
        RCLCPP_INFO(this->get_logger(),
            "Grid codes: %c %c %c | %c %c %c",
            gridCodes[0], gridCodes[1], gridCodes[2],
            gridCodes[3], gridCodes[4], gridCodes[5]
        );

        // 6. Create your “canvas” for drawing:
        // — BLACK background mode (your “dot screen”):
        cv::Mat dotImage = cv::Mat::zeros(image.size(), image.type());
        // — LIVE‑FEED background mode (camera image behind outlines):
        // cv::Mat dotImage = image.clone();

        // 7. Draw outlines & labels.
        for (auto &d : mergedDetections) {
            cv::circle(dotImage, d.centroid, static_cast<int>(d.radius), d.color, 3);
            cv::putText(dotImage,
                        d.label,
                        d.centroid + cv::Point2f(-d.radius * 0.5f, d.radius + 20),
                        cv::FONT_HERSHEY_SIMPLEX, 0.7, d.color, 2);
        }

        // 8. Display the live results.
        cv::imshow("Camera View", image);
        cv::imshow("Bottle Type", dotImage);
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RealSenseCaptureNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
