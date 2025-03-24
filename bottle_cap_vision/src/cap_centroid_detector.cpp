#include <opencv2/opencv.hpp>
#include <iostream>

int main() {
    // Load the image
    cv::Mat image = cv::imread("/home/luis/ros2_ws/src/bottle_cap_vision/test_images/test_bottle.jpeg");
    if (image.empty()) {
        std::cerr << "Error: Could not load image." << std::endl;
        return -1;
    }

    // Convert to HSV
    cv::Mat hsv;
    cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);

    // 1) Narrower color ranges
    cv::Mat blue_mask, red_mask1, red_mask2, red_mask, green_mask;

    // Blue
    cv::inRange(hsv, cv::Scalar(105, 150, 50), cv::Scalar(125, 255, 255), blue_mask);

    // Red
    cv::inRange(hsv, cv::Scalar(0, 150, 100), cv::Scalar(10, 255, 255), red_mask1);
    cv::inRange(hsv, cv::Scalar(170, 150, 100), cv::Scalar(180, 255, 255), red_mask2);
    red_mask = red_mask1 | red_mask2;

    // Green
    cv::inRange(hsv, cv::Scalar(45, 150, 50), cv::Scalar(75, 255, 255), green_mask);

    // 2) Morphological opening for each mask
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5,5));

    cv::erode(blue_mask, blue_mask, kernel);
    cv::dilate(blue_mask, blue_mask, kernel);

    cv::erode(red_mask, red_mask, kernel);
    cv::dilate(red_mask, red_mask, kernel);

    cv::erode(green_mask, green_mask, kernel);
    cv::dilate(green_mask, green_mask, kernel);


    // Helper lambda to detect and draw
    auto process_color = [&](cv::Mat mask, const std::string& label, const cv::Scalar& draw_color) {
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        for (size_t i = 0; i < contours.size(); ++i) {
            double area = cv::contourArea(contours[i]);
            if (area < 500) continue;

            cv::Moments M = cv::moments(contours[i]);
            if (M.m00 > 0) {
                int cx = static_cast<int>(M.m10 / M.m00);
                int cy = static_cast<int>(M.m01 / M.m00);
                cv::circle(image, cv::Point(cx, cy), 7, draw_color, -1);
                cv::putText(image, label, cv::Point(cx + 10, cy), cv::FONT_HERSHEY_SIMPLEX, 0.5, draw_color, 2);
                std::cout << label << " cap at (" << cx << ", " << cy << ")\n";
            }
        }
    };

    // Process each color
    process_color(red_mask, "Red", cv::Scalar(0, 0, 255));
    process_color(blue_mask, "Blue", cv::Scalar(255, 0, 0));
    process_color(green_mask, "Green", cv::Scalar(0, 255, 0));

    // Show result
    cv::imshow("Detected Bottle Caps", image);
    cv::waitKey(0);
    return 0;
}
