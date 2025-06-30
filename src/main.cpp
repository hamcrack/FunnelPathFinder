#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <algorithm>
#include <limits>
#include <iomanip>
#include <sstream>
#include <cassert>

#include <opencv2/opencv.hpp>
#include <geos_c.h>
#include <geos/geom/Coordinate.h>
#include <geos/algorithm/Orientation.h>

#include "json.hpp"
using json = nlohmann::json;

std::string videoFilename = "funnel_visualization.avi";

// Define a structure to hold the data point
struct DataPoint {
    std::string label;
    double x;
    double y;
};

// Implicit function to deserialize JSON into DataPoint
void from_json(const json& j, DataPoint& p) {
    j.at("label").get_to(p.label);
    j.at("x").get_to(p.x);
    j.at("y").get_to(p.y);
}

int geos_orientation(const cv::Point& p1, const cv::Point& p2, const cv::Point& p3) {
    // Convert cv::Point to GEOS's Coordinate type
    geos::geom::Coordinate c1(p1.x, p1.y);
    geos::geom::Coordinate c2(p2.x, p2.y);
    geos::geom::Coordinate c3(p3.x, p3.y);

    // Call the robust, official GEOS algorithm
    return geos::algorithm::Orientation::index(c1, c2, c3);
}

void visualize_step(const cv::Mat& baseImage,
                    const std::vector<cv::Point>& path,
                    const cv::Point& apex,
                    const cv::Point& left_tentacle_tip,
                    const cv::Point& right_tentacle_tip,
                    const cv::Point& point_being_tested,
                    const std::string& message,
                    cv::VideoWriter* videoWriter = nullptr) 
{
    cv::Mat frame = baseImage.clone();

    // Draw the funnel tentacles
    cv::line(frame, apex, left_tentacle_tip, cv::Scalar(0, 255, 255), 1, cv::LINE_AA); // Yellow
    cv::line(frame, apex, right_tentacle_tip, cv::Scalar(0, 255, 255), 1, cv::LINE_AA); // Yellow

    // Draw the line being tested
    cv::line(frame, apex, point_being_tested, cv::Scalar(255, 0, 255), 2, cv::LINE_AA); // Magenta

    // Draw the path
    for (size_t i = 0; i < path.size() - 1; ++i) {
        cv::line(frame, path[i], path[i+1], cv::Scalar(0, 255, 0), 2, cv::LINE_AA); // Green
    }
    cv::putText(frame, message, cv::Point(10, 20), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);

    if (videoWriter) {
        videoWriter->write(frame); // Write frame to video
    }

    cv::imshow("Funnel Algorithm Visualization", frame);
    cv::waitKey(500);
}

int main() {
    std::cout << "Initializing project..." << std::endl;
    std::cout << "OpenCV version: " << CV_VERSION << std::endl;

    initGEOS(nullptr, nullptr);
    const char* geos_version = GEOSversion();
    std::cout << "GEOS version: " << geos_version << std::endl;
    std::cout << "\nSuccessfully linked GEOS and OpenCV!" << std::endl;
    
    // --- JSON Reading Section ---
    std::string jsonFilePath = "../data.json";
    std::ifstream inputFile(jsonFilePath);

    if (!inputFile.is_open()) {
        std::cerr << "Error: Could not open JSON file: " << jsonFilePath << std::endl;
        finishGEOS();
        return 1; 
    }

    std::vector<DataPoint> dataPoints;
    try {
        json j;
        inputFile >> j;
        dataPoints = j.get<std::vector<DataPoint>>();
    } catch (const json::exception& e) {
        std::cerr << "JSON error: " << e.what() << std::endl;
        finishGEOS();
        return 1;
    }
    inputFile.close();

    // --- Image Plotting Setup ---
    cv::Mat image = cv::Mat::zeros(600, 800, CV_8UC3);
    if (dataPoints.empty()) {
        std::cout << "No data points to plot" << std::endl;
        finishGEOS();
        return 1;
    }

    double minX = std::numeric_limits<double>::max();
    double maxX = std::numeric_limits<double>::lowest();
    double minY = std::numeric_limits<double>::max();
    double maxY = std::numeric_limits<double>::lowest();
    for (const auto& point : dataPoints) {
        minX = std::min(minX, point.x);
        maxX = std::max(maxX, point.x);
        minY = std::min(minY, point.y);
        maxY = std::max(maxY, point.y);
    }

    double dataRangeX = maxX - minX;
    double dataRangeY = maxY - minY;
    if (dataRangeX == 0) dataRangeX = 2.0;
    if (dataRangeY == 0) dataRangeY = 2.0;
    int border = 50;
    int plotWidth = image.cols - 2 * border;
    int plotHeight = image.rows - 2 * border;
    double scale = std::min(static_cast<double>(plotWidth) / dataRangeX, static_cast<double>(plotHeight) / dataRangeY);
    double scaledDataWidth = dataRangeX * scale;
    double scaledDataHeight = dataRangeY * scale;
    double offsetX = border + (plotWidth - scaledDataWidth) / 2.0;
    double offsetY = border + (plotHeight - scaledDataHeight) / 2.0;

    std::vector<cv::Point> leftPixelPoints;
    std::vector<cv::Point> rightPixelPoints;
    
    // Sort datapoints to ensure FROM is first and TO is last for correct processing.
    std::sort(dataPoints.begin(), dataPoints.end(), [](const DataPoint& a, const DataPoint& b) {
        if (a.label == "FROM") return true;
        if (b.label == "FROM") return false;
        if (a.label == "TO") return false;
        if (b.label == "TO") return true;
        return a.label < b.label;
    });

    DataPoint fromPoint, toPoint;
    std::vector<std::pair<DataPoint, DataPoint>> gateways;

    for(const auto& p : dataPoints) {
        if(p.label == "FROM") fromPoint = p;
        else if (p.label == "TO") toPoint = p;
        else {
            if (gateways.empty() || gateways.back().second.label != "") {
                gateways.push_back({p, {"", 0, 0}});
            } else {
                gateways.back().second = p;
            }
        }
        // show lable and coordinates on the image
        cv::Point pixel(static_cast<int>(offsetX + (p.x - minX) * scale),
                        static_cast<int>(offsetY + scaledDataHeight - (p.y - minY) * scale) - 10);
        cv::putText(image, p.label, pixel, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
    }
    
    // Create the left and right chains for the funnel algorithm
    auto map_to_pixel = [&](const DataPoint& p) {
        int px = static_cast<int>(offsetX + (p.x - minX) * scale);
        int py = static_cast<int>(offsetY + scaledDataHeight - (p.y - minY) * scale);
        return cv::Point(px, py);
    };

    leftPixelPoints.push_back(map_to_pixel(fromPoint));
    rightPixelPoints.push_back(map_to_pixel(fromPoint));

    for(const auto& g : gateways) {
        leftPixelPoints.push_back(map_to_pixel(g.first));
        rightPixelPoints.push_back(map_to_pixel(g.second));
    }

    leftPixelPoints.push_back(map_to_pixel(toPoint));
    rightPixelPoints.push_back(map_to_pixel(toPoint));

    // Create a base image with all points and gateways drawn
    for (size_t i = 0; i < leftPixelPoints.size(); ++i) {
        if (i > 0 && i < leftPixelPoints.size() - 1) { // Don't draw lines for FROM/TO
             cv::line(image, leftPixelPoints[i], rightPixelPoints[i], cv::Scalar(255, 0, 0), 2); // Blue gateways
        }
        cv::circle(image, leftPixelPoints[i], 5, cv::Scalar(0, 0, 255), -1); // Red circles
        cv::circle(image, rightPixelPoints[i], 5, cv::Scalar(0, 0, 255), -1); // Red circles
    }

    // Init video writer for visualization
    cv::VideoWriter videoWriter;
    int fourcc = cv::VideoWriter::fourcc('M','J','P','G');
    double fps = 2.0; // 2 frames per second, adjust as needed
    cv::Size frameSize(image.cols, image.rows);
    videoWriter.open(videoFilename, fourcc, fps, frameSize, true);

    if (!videoWriter.isOpened()) {
        std::cerr << "Could not open the output video file for write\n";
        finishGEOS();
        return 1;
    }

    // --- Funnel Algorithm with Visualization ---
    std::vector<cv::Point> path;
    if (leftPixelPoints.empty()) {
        std::cerr << "No points to process for pathfinding." << std::endl;
        finishGEOS();
        return 1;
    }

    path.push_back(leftPixelPoints.front());
    int apex_idx = 0;
    int left_idx = 1;
    int right_idx = 1;
    int n = leftPixelPoints.size();

    while (apex_idx < n - 1) {
        cv::Point apex_point = path.back();

        // Find the starting indices for the tentacles from the current apex
        left_idx = apex_idx + 1;
        right_idx = apex_idx + 1;

        // Iterate through the remaining gateways to update the funnel
        for (int i = apex_idx + 1; i < n; ++i) {
            // -- Check Right Side of Funnel --
            visualize_step(image, path, apex_point, leftPixelPoints[left_idx], rightPixelPoints[right_idx], rightPixelPoints[i], "Testing Right Tentacle", &videoWriter);
            
            // If the new right point crosses over the left tentacle, the funnel must tighten.
            if (geos_orientation(apex_point, leftPixelPoints[left_idx], rightPixelPoints[i]) != 1) {
                 // The new apex is the tip of the left tentacle, just before the crossover.
                 apex_point = leftPixelPoints[left_idx];
                 path.push_back(apex_point);
                 
                 // Find the index of our new apex.
                 auto it = std::find(leftPixelPoints.begin(), leftPixelPoints.end(), apex_point);
                 apex_idx = std::distance(leftPixelPoints.begin(), it);
                 
                 visualize_step(image, path, path[path.size()-2], apex_point, apex_point, apex_point, "Right would cross left! New apex on left.", &videoWriter);
                 break; // Restart the main loop
            } else if (geos_orientation(apex_point, rightPixelPoints[right_idx], rightPixelPoints[i]) != 1) {
                right_idx = i; // Update the right tentacle tip if the new point is collinear or to the right
                visualize_step(image, path, apex_point, leftPixelPoints[left_idx], rightPixelPoints[right_idx], rightPixelPoints[i], "Right Tentacle Update", &videoWriter);
            }

            // -- Check Left Side of Funnel --
            visualize_step(image, path, apex_point, leftPixelPoints[left_idx], rightPixelPoints[right_idx], leftPixelPoints[i], "Testing Left Tentacle", &videoWriter);

            // If the new left point crosses over the right tentacle, the funnel must tighten.
            if (geos_orientation(apex_point, rightPixelPoints[right_idx], leftPixelPoints[i]) != -1) {
                // The new apex is the tip of the right tentacle,
                apex_point = rightPixelPoints[right_idx];
                path.push_back(apex_point); 

                // Find the index of our new apex.
                auto it = std::find(rightPixelPoints.begin(), rightPixelPoints.end(), apex_point);
                apex_idx = std::distance(rightPixelPoints.begin(), it); 
                visualize_step(image, path, path[path.size()-2], apex_point, apex_point, apex_point, "Left would cross right! New apex on right.", &videoWriter);
                break; // Restart the main loop
            } else if (geos_orientation(apex_point, leftPixelPoints[left_idx], leftPixelPoints[i]) != -1) { // Not a right turn
                left_idx = i; // Update the left tentacle
                visualize_step(image, path, apex_point, leftPixelPoints[left_idx], rightPixelPoints[right_idx], leftPixelPoints[i], "Left Tentacle Update", &videoWriter);
            }
            
            // If we have checked all points without finding a new apex, the path is straight to the end.
            if (i == n - 1) {
                apex_idx = n - 1; 
            }
        }
    }

    // Add the final destination point if it's not already the last point in the path.
    if (path.back().x != leftPixelPoints.back().x || path.back().y != leftPixelPoints.back().y) {
        path.push_back(leftPixelPoints.back());
    }

    // --- Final Drawing ---
    std::cout << "\n--- Shortest Path Found ---" << std::endl;
    for (size_t i = 0; i < path.size() - 1; ++i) {
        std::cout << "Segment " << i << ": (" << path[i].x << ", " << path[i].y << ") -> (" << path[i+1].x << ", " << path[i+1].y << ")" << std::endl;
        cv::line(image, path[i], path[i+1], cv::Scalar(0, 255, 0), 2, cv::LINE_AA); // Draw final path in green
    }
    std::cout << "--------------------------" << std::endl;

    cv::imshow("Final Shortest Path", image);
    cv::waitKey(0); // Wait indefinitely for a key press

    cv::imwrite("shortest_path_result.png", image);
    std::cout << "Generated a final image: 'shortest_path_result.png'" << std::endl;

    videoWriter.release();
    std::cout << "Saved funnel visualization video: " << videoFilename << std::endl;

    finishGEOS();
    return 0;
}