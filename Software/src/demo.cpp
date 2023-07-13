#include <open

int main()
[
    cv::Mat target = cv::imread("target.jpg"); //cv::IMREAD_COLOR
    cv::imshow("output",target);
    cv::waitKey(0);
]