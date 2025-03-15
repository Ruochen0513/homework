#include <iostream>
#include "opencv2/opencv.hpp"

using namespace std;

int main()
{
    cv::Mat img = cv::imread("test.jpg");
    cv::resize(img, img, cv::Size(600, 400));
    cv::imshow("image", img);
    cv::waitKey(0);
    return 0;
}