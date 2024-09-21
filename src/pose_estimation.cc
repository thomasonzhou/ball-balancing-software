#include "pose_estimation.h"
#include <iostream>

void generateMarker(){
    cv::Mat markerImage;
    cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    cv::aruco::generateImageMarker(dictionary, 23, 200, markerImage, 1);
    cv::imwrite("marker23.png", markerImage);
}

int main(){
    std::cout << "Starting pose estimation"<<std::endl;
    generateMarker();
}

