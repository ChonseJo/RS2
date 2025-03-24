#include <opencv2/opencv.hpp>
#include <iostream>

// Global images
cv::Mat image;      // Original BGR image
cv::Mat hsvImage;   // HSV version of the image

// Mouse callback function
void onMouse(int event, int x, int y, int flags, void* userdata)
{
    // Debug: print that the callback was triggered
    std::cout << "Mouse event: " << event << " at (" << x << ", " << y << ")\n";
    
    if (event == cv::EVENT_LBUTTONDOWN) {
        if (x < image.cols && y < image.rows) {
            cv::Vec3b hsvPixel = hsvImage.at<cv::Vec3b>(y, x);
            std::cout << "Clicked at (" << x << ", " << y << "): HSV = ("
                      << static_cast<int>(hsvPixel[0]) << ", " 
                      << static_cast<int>(hsvPixel[1]) << ", " 
                      << static_cast<int>(hsvPixel[2]) << ")\n";
        }
    }
}

int main()
{
    // Load the image using an absolute path
    image = cv::imread("/home/luis/ros2_ws/src/bottle_cap_vision/test_images/test_bottle.jpeg");
    if (image.empty()) {
        std::cerr << "Error: Could not load image." << std::endl;
        return -1;
    }
    
    // Convert to HSV
    cv::cvtColor(image, hsvImage, cv::COLOR_BGR2HSV);
    
    // Create window and set mouse callback
    cv::namedWindow("Image", cv::WINDOW_AUTOSIZE);
    cv::setMouseCallback("Image", onMouse, nullptr);
    
    std::cout << "Click on the image window to get HSV values. Press ESC to exit." << std::endl;
    
    // Display the image until ESC is pressed
    while (true) {
        cv::imshow("Image", image);
        int key = cv::waitKey(30);
        if (key == 27) break;  // ESC key
    }
    
    cv::destroyAllWindows();
    return 0;
}
