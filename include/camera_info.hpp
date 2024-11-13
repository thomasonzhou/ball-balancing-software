#ifndef CAMERA_INFO_H_
#define CAMERA_INFO_H_

#include <opencv2/objdetect/charuco_detector.hpp>
#include <opencv2/opencv.hpp>
const int ARUCOTAG_DICTIONARY = cv::aruco::DICT_6X6_250;
const std::string CAMERA_PARAMS_FILE = "iphone13p_calib.yml";
// const std::string CAMERA_PARAMS_FILE = "macbook_calib.yml";

// Charuco board specs
const int squaresX = 5;
const int squaresY = 7;

constexpr double CHARUCO_SQUARE_PIXELS = 100;
constexpr double CHARUCO_MARKER_PIXELS = 60;

constexpr float MARKER_LENGTH_INCHES = 2.0;
constexpr float MARKER_LENGTH_METERS =
    MARKER_LENGTH_INCHES * 0.0254;  // in meters

constexpr int MARKER_NUMBER = 23;
constexpr double CAMERA_TO_SHAFT_HEIGHT = 0.06;  // meters

#endif
