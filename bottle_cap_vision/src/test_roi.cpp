#include <opencv2/opencv.hpp>
#include <iostream>
using namespace cv;

int main() {
  // 1. Load your iPad box image
  Mat image = imread("/home/luis/ros2_ws/src/bottle_cap_vision/test_images/ipad_box.png");
  if (image.empty()) {
    std::cerr << "Failed to load test image\n";
    return -1;
  }

  // 2. Copy‐and‐paste the ROI+grid code **inside** this block:
  const int x0 = 100, y0 =  50;
  const int x1 = 700, y1 = 450;
  const int cols = 3, rows = 2;
  int cellW = (x1-x0)/cols, cellH = (y1-y0)/rows;

  Mat canvas = Mat::zeros(image.size(), image.type());
  // draw vertical lines
  for(int c=0;c<=cols;++c){
    int x = x0 + c*cellW;
    line(canvas, Point(x,y0), Point(x,y1), Scalar(255,255,255), 1);
  }
  // draw horizontal lines
  for(int r=0;r<=rows;++r){
    int y = y0 + r*cellH;
    line(canvas, Point(x0,y), Point(x1,y), Scalar(255,255,255), 1);
  }

  // 3. Show it
  imshow("Test Image", image);
  imshow("ROI Grid Overlay", canvas);
  waitKey(0);
  return 0;
}
