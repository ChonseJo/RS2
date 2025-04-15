#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <string>

struct CapDetection {
    cv::Point2f centroid;
    std::string label;
    cv::Scalar color; // BGR
};

int main()
{
    // 1. Load the snapshot image
    cv::Mat image = cv::imread("/home/luis/ros2_ws/src/bottle_cap_vision/test_images/test_bottle.jpeg");
    if (image.empty()) {
        std::cerr << "Error: Could not load image." << std::endl;
        return -1;
    }

    // 2. Convert to HSV
    cv::Mat hsv;
    cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);

    // 3. Define masks for red, green, blue
    cv::Mat red_mask;
    cv::Mat green_mask, blue_mask;

    // Red
    cv::inRange(hsv, cv::Scalar(0, 180, 205), cv::Scalar(10, 200, 230), red_mask);
    // red_mask = red_mask1 | red_mask2;

    // Green 
    cv::inRange(hsv, cv::Scalar(50, 60, 50), cv::Scalar(70, 80, 150), green_mask);

    // Blue 
    cv::inRange(hsv, cv::Scalar(105, 150, 50), cv::Scalar(125, 255, 255), blue_mask);

    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5,5));
    cv::morphologyEx(red_mask, red_mask, cv::MORPH_OPEN, kernel);
    cv::morphologyEx(green_mask, green_mask, cv::MORPH_CLOSE, kernel);
    cv::morphologyEx(blue_mask, blue_mask, cv::MORPH_OPEN, kernel);

    // 4. Find contours & compute centroids for each color
    auto find_centroids = [&](const cv::Mat& mask, const std::string& label, cv::Scalar color) {
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        std::vector<CapDetection> detections;
        for (size_t i = 0; i < contours.size(); ++i) {
            double area = cv::contourArea(contours[i]);
            if (area < 500) continue; // skip small noise

            cv::Moments M = cv::moments(contours[i]);
            if (M.m00 != 0) {
                cv::Point2f c(M.m10 / M.m00, M.m01 / M.m00);
                detections.push_back({c, label, color});
            }
        }
        return detections;
    };

    std::vector<CapDetection> allDetections;
    // Red caps
    auto redDet = find_centroids(red_mask, "Red", cv::Scalar(0, 0, 255));
    allDetections.insert(allDetections.end(), redDet.begin(), redDet.end());

    // Green caps
    auto greenDet = find_centroids(green_mask, "Green", cv::Scalar(0, 255, 0));
    allDetections.insert(allDetections.end(), greenDet.begin(), greenDet.end());

    // Blue caps
    auto blueDet = find_centroids(blue_mask, "Blue", cv::Scalar(255, 0, 0));
    allDetections.insert(allDetections.end(), blueDet.begin(), blueDet.end());

    // 5. Create a black background image (same size as original)
    cv::Mat dotImage = cv::Mat::zeros(image.size(), image.type());

    // 6. Draw the colored dots
    for (auto &d : allDetections) {
        cv::circle(dotImage, d.centroid, 40, d.color, -1); // solid circle
        // Optionally label them
        cv::putText(dotImage, d.label, d.centroid + cv::Point2f(-15,70),
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, d.color, 2);
    }

    // Show results
    cv::imshow("Original", image);
    cv::imshow("Dot Representation", dotImage);
    cv::waitKey(0);
    cv::destroyAllWindows();

    return 0;
}


