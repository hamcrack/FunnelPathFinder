#include <iostream>
#include <fstream>
#include <vector>
#include <string>

#include <opencv2/opencv.hpp>
#include <geos_c.h>

#include "json.hpp" // Include the nlohmann/json header
using json = nlohmann::json;

struct DataPoint {
    std::string label;
    double x;
    double y;
};

// Function to deserialize JSON into DataPoint
void from_json(const json& j, DataPoint& p) {
    j.at("label").get_to(p.label);
    j.at("x").get_to(p.x);
    j.at("y").get_to(p.y);
}


int main() {
    std::cout << "Initializing project..." << std::endl;
    std::cout << "OpenCV version: " << CV_VERSION << std::endl;

    // Initialize a context handle for thread safety.
    initGEOS(nullptr, nullptr);
    const char* geos_version = GEOSversion();
    std::cout << "GEOS version: " << geos_version << std::endl;

    std::cout << "\nSuccessfully linked GEOS and OpenCV!" << std::endl;
    
    // --- JSON Reading Section ---
    std::string jsonFilePath = "../data.json"; // Path to your JSON file
    std::ifstream inputFile(jsonFilePath);

    if (!inputFile.is_open()) {
        std::cerr << "Error: Could not open JSON file: " << jsonFilePath << std::endl;
        finishGEOS();
        return 1; 
    }

    try {
        json j;
        inputFile >> j; // Parse JSON directly from the input file stream

        // Convert the JSON array to a vector of DataPoint objects
        std::vector<DataPoint> dataPoints = j.get<std::vector<DataPoint>>();

        std::cout << "\n--- Parsed JSON Data ---" << std::endl;
        for (const auto& point : dataPoints) {
            std::cout << "Label: " << point.label
                      << ", X: " << point.x
                      << ", Y: " << point.y << std::endl;
        }
        std::cout << "------------------------" << std::endl;

    } catch (const json::parse_error& e) {
        std::cerr << "JSON parse error: " << e.what() << " at byte " << e.byte << std::endl;
        finishGEOS();
        return 1;
    } catch (const json::exception& e) {
        std::cerr << "JSON error: " << e.what() << std::endl;
        finishGEOS();
        return 1;
    }

    inputFile.close();

    cv::Mat image = cv::Mat::zeros(400, 600, CV_8UC3);
    cv::putText(image, "Setup Successful!", cv::Point(150, 200), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2);
    cv::imwrite("setup_test.png", image);
    std::cout << "Generated a test image: 'setup_test.png'" << std::endl;

    finishGEOS();

    return 0;
}