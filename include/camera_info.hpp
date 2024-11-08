#ifndef CAMERA_INFO_H_
#define CAMERA_INFO_H_

#include <opencv2/objdetect/charuco_detector.hpp> 
#include <opencv2/opencv.hpp>
const int ARUCOTAG_DICTIONARY = cv::aruco::DICT_6X6_250;
const std::string CAMERA_PARAMS_FILE = "macbook_calib.yml";

// Charuco board specs
const int squaresX = 5;
const int squaresY = 7;
constexpr int squareLength = 100;
constexpr int markerLength = 60;
#endif
