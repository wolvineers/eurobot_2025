#include <opencv2/opencv.hpp>
#include <iostream>

int main() {
    std::cout << "OpenCV version: " << CV_VERSION << std::endl;

    // Opcional: crea una ventana vacía
    cv::Mat image(240, 320, CV_8UC3, cv::Scalar(100, 150, 200));
    cv::imshow("Test Window", image);
    cv::waitKey(0);
    return 0;
}
