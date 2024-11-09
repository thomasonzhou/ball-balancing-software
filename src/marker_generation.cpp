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


    cv::aruco::CharucoBoard board(cv::Size(squaresX, squaresY), CHARUCO_SQUARE_PIXELS, CHARUCO_MARKER_PIXELS, dictionary);

    const double borderBits = 1.0;
    constexpr double margins = (CHARUCO_SQUARE_PIXELS - CHARUCO_MARKER_PIXELS) / 2.0;

    cv::Mat boardImage;
    cv::Size imageSize;
    imageSize.width = double(squaresX) * CHARUCO_MARKER_PIXELS + 2.0 * margins;
    imageSize.height = double(squaresY) * CHARUCO_MARKER_PIXELS + 2.0 * margins;
    board.generateImage(imageSize, boardImage, margins, borderBits);
    cv::imwrite("charuco_board.jpg", boardImage);
}

int main(){
    generateArucoMarker();
    generateCharucoMarkerBoard();
}
