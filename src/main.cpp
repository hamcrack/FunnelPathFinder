#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <algorithm>
#include <limits>
#include <iomanip>
#include <sstream>

#include <opencv2/opencv.hpp>
#include <geos_c.h>
#include <geos/geom/Coordinate.h>
#include <geos/algorithm/Orientation.h>

#include "json.hpp"
using json = nlohmann::json;

std::string jsonFilePath = "../data.json";
int visualizationDelay = 500; // (milliseconds) make 0 for manual step-by-step visualization wich key press
std::string imageFilename = "shortest_path_result.png";
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

// Function to convert label to an index (A=1, B=2, ..., Z=26, AA=27, AB=28, ...)
int labelToIndex(const std::string& label) {
    int result = 0;
    for (char c : label) {
        result *= 26;
        result += (std::toupper(c) - 'A' + 1);
    }
    return result;
}

// Replacement for the cross product
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
                    const std::vector<cv::Point>& left_funnel,
                    const std::vector<cv::Point>& right_funnel,
                    const cv::Point& point_being_tested,
                    const std::string& message,
                    cv::VideoWriter* videoWriter = nullptr) 
{
    cv::Mat frame = baseImage.clone();

    // Draw the funnels
    if (left_funnel.size() > 0) {
        cv::line(frame, path.back(), left_funnel[0], cv::Scalar(0, 255, 255), 1, cv::LINE_AA); // Yellow
    }
    if (right_funnel.size() > 0) {
        cv::line(frame, path.back(), right_funnel[0], cv::Scalar(0, 255, 255), 1, cv::LINE_AA); // Yellow
    }

    if (left_funnel.size() > 1) {
        for (size_t i = 0; i < left_funnel.size() - 1; ++i) {
            cv::line(frame, left_funnel[i], left_funnel[i + 1], cv::Scalar(0, 255, 255), 1, cv::LINE_AA); // Yellow
        }
    }
    if (right_funnel.size() > 1) {
        for (size_t i = 0; i < right_funnel.size() - 1; ++i) {
            cv::line(frame, right_funnel[i], right_funnel[i + 1], cv::Scalar(0, 255, 255), 1, cv::LINE_AA); // Yellow
        }
    }

    // Draw the line being tested
    cv::line(frame, path.back(), point_being_tested, cv::Scalar(255, 0, 255), 2, cv::LINE_AA); // Magenta
    cv::circle(frame, path.back(), 5, cv::Scalar(255, 255, 255), -1); // White circle for apex

    // Draw the path
    for (size_t i = 0; i < path.size() - 1; ++i) {
        cv::line(frame, path[i], path[i + 1], cv::Scalar(0, 255, 0), 2, cv::LINE_AA); // Green
    }
    cv::putText(frame, message, cv::Point(10, 20), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);

    if (videoWriter) {
        videoWriter->write(frame); // Write frame to video
    }

    cv::imshow("Funnel Algorithm Visualization", frame);
    cv::waitKey(visualizationDelay);
}

int main() {
    std::cout << "Initializing project..." << std::endl;
    std::cout << "OpenCV version: " << CV_VERSION << std::endl;

    initGEOS(nullptr, nullptr);
    const char* geos_version = GEOSversion();
    std::cout << "GEOS version: " << geos_version << std::endl;
    std::cout << "\nSuccessfully linked GEOS and OpenCV!" << std::endl;
    
    // --- JSON Reading Section ---
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

    // --- Calculate Data Limits for Scaling ---
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

    auto map_to_pixel = [&](const DataPoint& p) {
        int px = static_cast<int>(offsetX + (p.x - minX) * scale);
        int py = static_cast<int>(offsetY + scaledDataHeight - (p.y - minY) * scale);
        return cv::Point(px, py);
    };

    // --- Robust Data Preparation ---
    std::vector<cv::Point> leftPixelPoints;
    std::vector<cv::Point> rightPixelPoints;
    DataPoint fromPoint, toPoint;
    std::vector<DataPoint> gatewayPoints;

    // Separate FROM, TO, and gateway points
    for (const auto& p : dataPoints) {
        if (p.label == "FROM") {
            fromPoint = p;
        } else if (p.label == "TO") {
            toPoint = p;
        } else {
            gatewayPoints.push_back(p);
        }
    }

    // Sort gateway points alphabetically by label to ensure correct order (A, B, C, D...)
    std::sort(gatewayPoints.begin(), gatewayPoints.end(), [](const DataPoint& a, const DataPoint& b) {
        return labelToIndex(a.label) < labelToIndex(b.label);
    });

    // Build the final left and right chains in the correct order
    leftPixelPoints.push_back(map_to_pixel(fromPoint));
    rightPixelPoints.push_back(map_to_pixel(fromPoint));

    for (size_t i = 0; i < gatewayPoints.size(); i += 2) {
        // Assuming labels are paired correctly (A then B, C then D) after sorting
        leftPixelPoints.push_back(map_to_pixel(gatewayPoints[i]));
        rightPixelPoints.push_back(map_to_pixel(gatewayPoints[i+1]));
    }

    leftPixelPoints.push_back(map_to_pixel(toPoint));
    rightPixelPoints.push_back(map_to_pixel(toPoint));

    // --- Draw the base image with points and gateways ---
    for (size_t i = 1; i < leftPixelPoints.size() - 1; ++i) {
        cv::line(image, leftPixelPoints[i], rightPixelPoints[i], cv::Scalar(255, 0, 0), 2); // Blue gateways
    }
    for (const auto& p : dataPoints) {
        cv::Point pixel = map_to_pixel(p);
        cv::circle(image, pixel, 5, cv::Scalar(0, 0, 255), -1); // Red circles
        cv::putText(image, p.label, cv::Point(pixel.x, pixel.y - 10), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
    }
    
    // --- Video Writer Setup ---
    cv::VideoWriter videoWriter;
    int fourcc = cv::VideoWriter::fourcc('M','J','P','G');
    double fps = 5.0; // Increased FPS for smoother video
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
    int n = leftPixelPoints.size();

    std::vector<cv::Point> leftFunnelPoints;
    std::vector<cv::Point> rightFunnelPoints;

    // Iterate through the gateways to update the funnel
    for (int i = 0; i < n; ++i) {

        // -- Check Left Side of Funnel --
        visualize_step(image, path, leftFunnelPoints, rightFunnelPoints, leftPixelPoints[i], "Processing left point...", &videoWriter);

        if (leftFunnelPoints.empty() && leftPixelPoints[i] != path.back()) {
            leftFunnelPoints.push_back(leftPixelPoints[i]);
        } else if (leftFunnelPoints.size() > 0 && leftFunnelPoints.back() != leftPixelPoints[i]) {
            if (geos_orientation(path.back(), leftFunnelPoints.back(), leftPixelPoints[i]) != geos::algorithm::Orientation::CLOCKWISE) {
                // the new left point is right of the last left point
                int last_collision = -1;
                for (int j = 0; j < rightFunnelPoints.size() - 1; ++j) {
                    if (geos_orientation(path.back(), rightFunnelPoints[j], leftPixelPoints[i]) != geos::algorithm::Orientation::CLOCKWISE) {
                        // The new point is to the right of the j'th right funnel point
                        path.push_back(rightFunnelPoints[j]);
                        last_collision = j;
                    }
                }

                if (last_collision >= 0) {
                    // Collision with a previous right points when narrowing funnel
                    leftFunnelPoints = { leftPixelPoints[i] };
                    rightFunnelPoints.erase(rightFunnelPoints.begin(), rightFunnelPoints.begin() + last_collision + 1);
                    visualize_step(image, path, leftFunnelPoints, rightFunnelPoints, leftPixelPoints[i], "Point is to the right and collision found -> Narrowed funnel", &videoWriter);
                } else {
                    // No collision, just add the new left point
                    leftFunnelPoints.push_back(leftPixelPoints[i]);
                    visualize_step(image, path, leftFunnelPoints, rightFunnelPoints, leftPixelPoints[i], "Point is to the right but no collision -> Added left point to funnel", &videoWriter);
                }
            
            } else {  // new point opens to the left so add it to the left funnel
                leftFunnelPoints.push_back(leftPixelPoints[i]);
                visualize_step(image, path, leftFunnelPoints, rightFunnelPoints, leftPixelPoints[i], "Point is opening to the left -> Added left point to funnel", &videoWriter);
            }
        }

        // -- Check Right Side of Funnel --
        visualize_step(image, path, leftFunnelPoints, rightFunnelPoints, rightPixelPoints[i], "Processing right point...", &videoWriter);

        if (rightFunnelPoints.empty() && rightPixelPoints[i] != path.back()) {
            rightFunnelPoints.push_back(rightPixelPoints[i]);
        } else if (rightFunnelPoints.size() > 0 && rightFunnelPoints.back() != rightPixelPoints[i]) {
            if (geos_orientation(path.back(), rightFunnelPoints.back(), rightPixelPoints[i]) != geos::algorithm::Orientation::COUNTERCLOCKWISE) {
                // the new right point is left of the last right point
                int last_collision = -1;
                for (int j = 0; j < leftFunnelPoints.size() - 1; ++j) {
                    if (geos_orientation(path.back(), leftFunnelPoints[j], rightPixelPoints[i]) != geos::algorithm::Orientation::COUNTERCLOCKWISE) {
                        // The new point is to the left of the j'th left funnel point
                        path.push_back(leftFunnelPoints[j]);
                        last_collision = j;
                    }
                }

                if (last_collision >= 0) {
                    // Collision with a previous left points when narrowing funnel
                    rightFunnelPoints = { rightPixelPoints[i] };
                    leftFunnelPoints.erase(leftFunnelPoints.begin(), leftFunnelPoints.begin() + last_collision + 1);
                    visualize_step(image, path, leftFunnelPoints, rightFunnelPoints, rightPixelPoints[i], "Point is to the left and collision found -> Narrowed funnel", &videoWriter);
                } else {
                    // No collision, just add the new right point
                    visualize_step(image, path, leftFunnelPoints, rightFunnelPoints, rightPixelPoints[i], "Point is to the left but no collision -> Added right point to funnel", &videoWriter);
                    rightFunnelPoints.push_back(rightPixelPoints[i]);
                }
            
            } else {  // new point opens to the right so add it to the right funnel
                rightFunnelPoints.push_back(rightPixelPoints[i]);
                visualize_step(image, path, leftFunnelPoints, rightFunnelPoints, rightPixelPoints[i], "Point is opening to the right -> Added right point to funnel", &videoWriter);
            }
        }
    }

    // Add the final destination point if it's not already the last point in the path.
    if (path.back() != leftPixelPoints.back()) {
        path.push_back(leftPixelPoints.back());
    }       
    
    // --- Final Drawing ---
    std::cout << "\n--- Shortest Path Found ---" << std::endl;

    rightFunnelPoints.erase(rightFunnelPoints.begin(), rightFunnelPoints.end());
    leftFunnelPoints.erase(leftFunnelPoints.begin(), leftFunnelPoints.end());
    visualize_step(image, path, leftFunnelPoints, rightFunnelPoints, leftPixelPoints.back(), "Shortest Path", &videoWriter);

    for (size_t i = 0; i < path.size() - 1; ++i) {
        std::cout << "Segment " << i << ": (" << path[i].x << ", " << path[i].y << ") -> (" << path[i+1].x << ", " << path[i+1].y << ")" << std::endl;
        cv::line(image, path[i], path[i+1], cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
    }
    std::cout << "--------------------------" << std::endl;

    cv::imshow("Final Shortest Path", image);
    cv::waitKey(0);

    cv::imwrite(imageFilename, image);
    std::cout << "Generated a final image: '" << imageFilename << "'" << std::endl;

    videoWriter.release();
    std::cout << "Saved funnel visualization video: " << videoFilename << std::endl;

    finishGEOS();
    return 0;
}