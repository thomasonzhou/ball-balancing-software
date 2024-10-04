#include "camera_info.hpp"
using namespace std;

void generateArucoMarker(){
    cv::Mat markerImage;
    cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(ARUCOTAG_DICTIONARY);
    cv::aruco::generateImageMarker(dictionary, 42, 200, markerImage, 1);
    cv::imwrite("aruco_marker42.png", markerImage);
}

void generateCharucoMarkerBoard(){
    cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(ARUCOTAG_DICTIONARY);


    cv::aruco::CharucoBoard board(cv::Size(squaresX, squaresY), squareLength, markerLength, dictionary);

    const int borderBits = 1;
    constexpr int margins = squareLength - markerLength;

    cv::Mat boardImage;
    cv::Size imageSize;
    imageSize.width = squaresX * squareLength + 2 * margins;
    imageSize.height = squaresY * squareLength + 2 * margins;
    board.generateImage(imageSize, boardImage, margins, borderBits);
    cv::imwrite("charuco_board.jpg", boardImage);
}

int main(){
    generateArucoMarker();
    generateCharucoMarkerBoard();
}
