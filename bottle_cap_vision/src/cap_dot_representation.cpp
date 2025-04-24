#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <string>

int main()
{
    // 1. Load the snapshot image.
    cv::Mat image = cv::imread("/home/luis/ros2_ws/src/bottle_cap_vision/test_images/test_bottle.jpeg");
    if (image.empty()) {
        std::cerr << "Error: Could not load image." << std::endl;
        return -1;
    }

    // 2. Convert the image to HSV.
    cv::Mat hsv;
    cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);

    // 3. Create color masks for red, green, and blue.
    cv::Mat red_mask, green_mask, blue_mask;
    // Red mask: using one range for simplicity.
    cv::inRange(hsv, cv::Scalar(0, 150, 205), cv::Scalar(10, 200, 230), red_mask);
    // Green mask.
    cv::inRange(hsv, cv::Scalar(50, 60, 50), cv::Scalar(70, 80, 150), green_mask);
    // Blue mask.
    cv::inRange(hsv, cv::Scalar(105, 150, 50), cv::Scalar(125, 255, 255), blue_mask);

    // 4. Apply morphological closing to fill the rings.
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(15,15));
    cv::morphologyEx(red_mask, red_mask, cv::MORPH_CLOSE, kernel);
    cv::morphologyEx(green_mask, green_mask, cv::MORPH_CLOSE, kernel);
    cv::morphologyEx(blue_mask, blue_mask, cv::MORPH_CLOSE, kernel);

    cv::Mat dilate_kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(15, 15));
    cv::dilate(red_mask, red_mask, dilate_kernel);

    // Optional: display the filled masks for debugging.
    cv::imshow("Red Mask", red_mask);
    // cv::imshow("Green Mask", green_mask);
    // cv::imshow("Blue Mask", blue_mask);
    cv::waitKey(0);
    cv::destroyWindow("Red Mask");
    cv::destroyWindow("Green Mask");
    cv::destroyWindow("Blue Mask");

    // 5. Smooth the masks with Gaussian Blur.
    cv::GaussianBlur(red_mask, red_mask, cv::Size(9,9), 2, 2);
    cv::GaussianBlur(green_mask, green_mask, cv::Size(9,9), 2, 2);
    cv::GaussianBlur(blue_mask, blue_mask, cv::Size(9,9), 2, 2);

    // 6. Set up HoughCircles parameters.
    double dp = 1;
    double minDist = red_mask.rows / 4.0; // minimum distance between circles.
    int param1 = 100;  // Canny edge detector threshold.
    int param2 = 50;   // Accumulator threshold for circle centers.
    int minRadius = 20;
    int maxRadius = 60;

    // 7. Detect circles on each mask.
    std::vector<cv::Vec3f> redCircles, greenCircles, blueCircles;
    cv::HoughCircles(red_mask, redCircles, cv::HOUGH_GRADIENT, dp, minDist, param1, param2, minRadius, maxRadius);
    cv::HoughCircles(green_mask, greenCircles, cv::HOUGH_GRADIENT, dp, minDist, param1, param2, minRadius, maxRadius);
    cv::HoughCircles(blue_mask, blueCircles, cv::HOUGH_GRADIENT, dp, minDist, param1, param2, minRadius, maxRadius);

    // Debug: Print number of circles detected.
    std::cout << "Red circles detected: " << redCircles.size() << std::endl;
    std::cout << "Green circles detected: " << greenCircles.size() << std::endl;
    std::cout << "Blue circles detected: " << blueCircles.size() << std::endl;

    // 8. Draw detected circles on a copy of the original image.
    cv::Mat output = image.clone();

    auto drawCircles = [&](const std::vector<cv::Vec3f>& circles, const cv::Mat& mask,
                             const cv::Scalar& color, const std::string& label) {
        for (const auto &circle : circles) {
            cv::Point center(cvRound(circle[0]), cvRound(circle[1]));
            int radius = cvRound(circle[2]);
            // Check that the center is within bounds and belongs to the mask.
            if (center.x >= 0 && center.x < mask.cols && center.y >= 0 && center.y < mask.rows) {
                if (mask.at<uchar>(center) > 0) {
                    cv::circle(output, center, 3, color, -1);      // Center dot.
                    cv::circle(output, center, radius, color, 3);    // Circle outline.
                    cv::putText(output, label, center, cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 2);
                }
            }
        }
    };

    drawCircles(redCircles, red_mask, cv::Scalar(0,0,255), "Red");
    drawCircles(greenCircles, green_mask, cv::Scalar(0,255,0), "Green");
    drawCircles(blueCircles, blue_mask, cv::Scalar(255,0,0), "Blue");

    // 9. Display the final output image.
    cv::imshow("Detected Colored Circles", output);
    cv::waitKey(0);
    cv::destroyAllWindows();

    return 0;
}
