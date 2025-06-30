#include <iostream>
#include <opencv2/opencv.hpp>
#include <geos_c.h>

int main() {
    std::cout << "Initializing project..." << std::endl;
    std::cout << "OpenCV version: " << CV_VERSION << std::endl;

    // Initialize a context handle for thread safety.
    initGEOS(nullptr, nullptr);
    const char* geos_version = GEOSversion();
    std::cout << "GEOS version: " << geos_version << std::endl;

    std::cout << "\nSuccessfully linked GEOS and OpenCV!" << std::endl;
   
    // As a bonus, let's create a blank image with OpenCV to show it works
    cv::Mat image = cv::Mat::zeros(400, 600, CV_8UC3);
    cv::putText(image, "Setup Successful!", cv::Point(150, 200), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2);
    cv::imwrite("setup_test.png", image);
    std::cout << "Generated a test image: 'setup_test.png'" << std::endl;

    // Clean up GEOS at the VERY END of the program
    finishGEOS();

    return 0;
}