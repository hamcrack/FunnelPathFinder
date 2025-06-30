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

#include "json.hpp"
using json = nlohmann::json;

// Define a structure to hold the data point
struct DataPoint {
    std::string label;
    double x;
    double y;
};

// Function to deserialize JSON into DataPoint implicitly 
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
    std::string jsonFilePath = "../data.json"; // Path to your JSON file relative to the executable's location
    std::ifstream inputFile(jsonFilePath);

    if (!inputFile.is_open()) {
        std::cerr << "Error: Could not open JSON file: " << jsonFilePath << std::endl;
        finishGEOS();
        return 1; 
    }

    std::vector<DataPoint> dataPoints;
    try {
        json j;
        inputFile >> j; // Parse JSON directly from the input file stream

        // Convert the JSON array to a vector of DataPoint objects
        dataPoints = j.get<std::vector<DataPoint>>();

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

    cv::Mat image = cv::Mat::zeros(400, 600, CV_8UC3); // Black image: Height 400, Width 600, 3 channels (BGR)

    if (dataPoints.empty()) {
        std::cout << "No data points to plot" << std::endl;
        cv::putText(image, "No data points to plot!", cv::Point(100, 200), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2);
    } else {
        // 1. Find Data Extents (min/max X, Y)
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

        // Extend range slightly to handle single point or collinear points and prevents division by zero
        if (dataRangeX == 0) {
            minX -= 1.0;
            maxX += 1.0;
            dataRangeX = 2.0;
        }
        if (dataRangeY == 0) {
            minY -= 1.0;
            maxY += 1.0;
            dataRangeY = 2.0;
        }

        // 2. Image Plotting
        int border = 30;
        int imgWidth = image.cols;
        int imgHeight = image.rows;

        int plotAreaLeft = border;
        int plotAreaRight = imgWidth - border;
        int plotAreaTop = border;
        int plotAreaBottom = imgHeight - border;

        int plotWidth = plotAreaRight - plotAreaLeft;
        int plotHeight = plotAreaBottom - plotAreaTop;

        // Ensure plot dimensions are positive
        if (plotWidth <= 0 || plotHeight <= 0) {
            std::cerr << "Error: Image dimensions too small for border. Cannot plot points." << std::endl;
            cv::putText(image, "Image too small for plotting!", cv::Point(100, 200), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2);
        } else {
            // 3. Calculate Scaling Factor (uniform scale to maintain aspect ratio)
            double scaleX = static_cast<double>(plotWidth) / dataRangeX;
            double scaleY = static_cast<double>(plotHeight) / dataRangeY;
            double scale = std::min(scaleX, scaleY);

            double scaledDataWidth = dataRangeX * scale;
            double scaledDataHeight = dataRangeY * scale;

            double offsetX = plotAreaLeft + (plotWidth - scaledDataWidth) / 2.0;
            double offsetY_top_aligned = plotAreaTop + (plotHeight - scaledDataHeight) / 2.0;

            // Store calculated pixel coordinates for efficiency
            std::vector<cv::Point> leftLetterPixelPoints;
            std::vector<cv::Point> rightLetterPixelPoints;

            // 5. Plot each point (circles and labels)
            int pointCnt = 0;
            for (const auto& point : dataPoints) {
                // Map data to pixel coordinates
                int pixelX = static_cast<int>(offsetX + (point.x - minX) * scale);
                int pixelY = static_cast<int>(offsetY_top_aligned + scaledDataHeight - (point.y - minY) * scale);

                assert(pixelX >= 0 && pixelX < imgWidth && pixelY >= 0 && pixelY < imgHeight && "Calculated pixel coordinates are out of image bounds!");
                
                cv::Point currentPixelPoint(pixelX, pixelY); // Create cv::Point

                // Draw point with labels
                cv::circle(image, currentPixelPoint, 5, cv::Scalar(0, 0, 255), -1); // Red circle
                std::string pointLabel = point.label;
                std::ostringstream coordsOss;
                coordsOss << std::fixed << std::setprecision(2) << "(" << point.x << ", " << point.y << ")";
                std::string coordsText = coordsOss.str();

                int fontFace = cv::FONT_HERSHEY_SIMPLEX;
                double labelFontScale = 0.4;
                double coordsFontScale = 0.35;
                int thickness = 1;

                cv::Size labelSize = cv::getTextSize(pointLabel, fontFace, labelFontScale, thickness, 0);
                cv::Size coordsSize = cv::getTextSize(coordsText, fontFace, coordsFontScale, thickness, 0);

                cv::Point labelPos(currentPixelPoint.x - labelSize.width / 2, currentPixelPoint.y - 10);
                labelPos.y = std::max(border + labelSize.height, labelPos.y);
                labelPos.x = std::max(border, labelPos.x);
                labelPos.x = std::min(imgWidth - border - labelSize.width, labelPos.x);
                cv::putText(image, pointLabel, labelPos, fontFace, labelFontScale, cv::Scalar(255, 255, 0), thickness);

                cv::Point coordsPos(currentPixelPoint.x - coordsSize.width / 2, currentPixelPoint.y + 20);
                coordsPos.y = std::min(imgHeight - border, coordsPos.y);
                coordsPos.x = std::max(border, coordsPos.x);
                coordsPos.x = std::min(imgWidth - border - coordsSize.width, coordsPos.x);
                cv::putText(image, coordsText, coordsPos, fontFace, coordsFontScale, cv::Scalar(0, 255, 255), thickness);

                bool isSpecialKeyword = (point.label == "FROM" || point.label == "TO");
                if (!isSpecialKeyword) {
                    if (pointCnt % 2 != 0) {
                        rightLetterPixelPoints.push_back(currentPixelPoint);
                    } else {
                        leftLetterPixelPoints.push_back(currentPixelPoint);
                    }
                }
                pointCnt++;
            }

            // 6. Connect gateways with blue lines
            size_t numPairsToConnect = std::min(leftLetterPixelPoints.size(), rightLetterPixelPoints.size());

            for (size_t i = 0; i < numPairsToConnect; ++i) {
                cv::Point p1 = leftLetterPixelPoints[i];
                cv::Point p2 = rightLetterPixelPoints[i];
                cv::line(image, p1, p2, cv::Scalar(255, 0, 0), 2);
            }
        }
    }

    cv::imwrite("setup_test.png", image);
    std::cout << "Generated a test image: 'setup_test.png' with plotted points." << std::endl;

    // Clean up GEOS at the VERY END of the program
    finishGEOS();

    return 0;
}