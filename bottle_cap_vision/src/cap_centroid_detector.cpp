#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>

int main() {
    // Load the image (using absolute path for robustness)
    cv::Mat image = cv::imread("/home/luis/ros2_ws/src/bottle_cap_vision/test_images/test_bottle.jpeg");
    if (image.empty()) {
        std::cerr << "Error: Could not load image." << std::endl;
        return -1;
    }

    // Convert to HSV
    cv::Mat hsv;
    cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);

    cv::Mat red_mask, green_mask, blue_mask;
    // Blue cap range (same in both codes)
    cv::inRange(hsv, cv::Scalar(105, 150, 50), cv::Scalar(125, 255, 255), blue_mask);
    // Red cap range (using the same HSV values as the ROS2 node)
    cv::inRange(hsv, cv::Scalar(0, 100, 100), cv::Scalar(10, 255, 255), red_mask);
    // Green cap range (adjusted to match the ROS2 node)
    cv::inRange(hsv, cv::Scalar(45, 65, 120), cv::Scalar(75, 255, 255), green_mask);

    // Combine all masks
    cv::Mat combined_mask = blue_mask | red_mask | green_mask;

    // Optional: clean the mask using morphological operations
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
    cv::morphologyEx(combined_mask, combined_mask, cv::MORPH_OPEN, kernel);

    // Find contours on the combined mask
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(combined_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    // Vector to hold the detected centroids
    std::vector<cv::Point2f> allCentroids;

    // Process each contour: filter by area and compute centroid
    for (size_t i = 0; i < contours.size(); ++i) {
        double area = cv::contourArea(contours[i]);
        if (area < 500) continue;  // adjust the threshold as needed

        cv::Moments M = cv::moments(contours[i]);
        if (M.m00 > 0) {
            int cx = static_cast<int>(M.m10 / M.m00);
            int cy = static_cast<int>(M.m01 / M.m00);
            allCentroids.push_back(cv::Point2f(cx, cy));
            // Draw the centroid on the image
            cv::circle(image, cv::Point(cx, cy), 5, cv::Scalar(0, 255, 0), -1);
        }
    }

    // Check that we have enough points to cluster
    if (allCentroids.empty()) {
        std::cerr << "No centroids detected." << std::endl;
        return -1;
    }

    int clusterCount = 6;  // desired number of clusters
    if (allCentroids.size() < clusterCount) {
        std::cout << "Detected only " << allCentroids.size() 
                  << " centroids. Adjusting cluster count to " << allCentroids.size() << std::endl;
        clusterCount = allCentroids.size();
    }
    
    cv::Mat pointsMat(allCentroids.size(), 2, CV_32F);
    for (size_t i = 0; i < allCentroids.size(); i++) {
        pointsMat.at<float>(i, 0) = allCentroids[i].x;
        pointsMat.at<float>(i, 1) = allCentroids[i].y;
    }
    
    cv::Mat labels, centers;
    // Run KMeans clustering
    cv::kmeans(pointsMat, clusterCount, labels,
               cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 10, 1.0),
               3, cv::KMEANS_PP_CENTERS, centers);

    // Draw the cluster centers (using white color)
    for (int i = 0; i < centers.rows; i++) {
        cv::Point centerPoint(static_cast<int>(centers.at<float>(i, 0)), static_cast<int>(centers.at<float>(i, 1)));
        cv::circle(image, centerPoint, 8, cv::Scalar(255, 255, 255), -1);
        std::cout << "Cluster " << i << " centroid: (" << centerPoint.x << ", " << centerPoint.y << ")" << std::endl;
    }

    // ----- PCA on Each Cluster (Optional) -----
    // For each cluster, collect points that belong to it
    for (int i = 0; i < clusterCount; i++) {
        std::vector<cv::Point2f> clusterPoints;
        for (size_t j = 0; j < allCentroids.size(); j++) {
            if (labels.at<int>(j, 0) == i) {
                clusterPoints.push_back(allCentroids[j]);
            }
        }
        if (clusterPoints.size() < 2) continue; // Need at least 2 points for PCA

        // Convert to Mat for PCA
        cv::Mat dataPts = cv::Mat(clusterPoints).reshape(1);
        dataPts.convertTo(dataPts, CV_32F);
        
        // Perform PCA
        cv::PCA pca_analysis(dataPts, cv::Mat(), cv::PCA::DATA_AS_ROW);
        cv::Point2f eigen_vec(pca_analysis.eigenvectors.at<float>(0, 0),
                              pca_analysis.eigenvectors.at<float>(0, 1));
        
        // Draw the principal component on the image from the cluster center
        cv::Point2f clusterCenter(static_cast<float>(centers.at<float>(i, 0)), static_cast<float>(centers.at<float>(i, 1)));
        cv::line(image, clusterCenter, clusterCenter + eigen_vec * 50, cv::Scalar(0, 0, 255), 2);
        std::cout << "Cluster " << i << " orientation vector: (" << eigen_vec.x << ", " << eigen_vec.y << ")" << std::endl;
    }

    // Show the final result
    cv::imshow("Detected Bottle Caps and Clusters", image);
    cv::waitKey(0);
    cv::destroyAllWindows();
    return 0;
}
