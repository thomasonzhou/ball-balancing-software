#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <iostream>

using namespace std;
using namespace cv;


const int cannyThresholdInitialValue = 102;
const int accumulatorThresholdInitialValue = 33;
const int maxCannyThreshold = 400;
const int maxAccumulatorThreshold = 200;
const string windowName = "Hough transform";
const string cannyTrackbarName = "Canny Edge Threshold";
const string accumulatorTrackbarName = "Accumulator Threshold";

void HoughDetection(const Mat& src_gray, Mat& src_display, int cannyThreshold, int accumulatorThreshold)
{
    std::vector<Vec3f> circles;
    HoughCircles(src_gray, circles, HOUGH_GRADIENT, 1, src_gray.rows / 8, cannyThreshold, accumulatorThreshold, 0, 0);

    //TODO: remove all circles that are too big/too small

    Mat display = src_display.clone();
    for (size_t i = 0; i < circles.size(); i++)
    {
        Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
        int radius = cvRound(circles[i][2]);
        circle(display, center, 3, Scalar(0, 255, 0), -1, 8, 0);
        circle(display, center, radius, Scalar(0, 0, 255), 3, 8, 0);
        cout << circles[i][0] << ", " << circles[i][1] << endl;
    }

    putText(display, "Circles detected: " + to_string(circles.size()), Point(10, 30),
            FONT_HERSHEY_SIMPLEX, 1.0, Scalar(255, 0, 0), 2);

    imshow(windowName, display);
}

int main(int argc, char** argv)
{
    Mat frame, src_gray, display;
    VideoCapture cap;

    cap.open(0);
    if (!cap.isOpened())
    {
        cerr << "Error: Could not open the default camera.\n";
        return -1;
    }

    namedWindow(windowName, WINDOW_AUTOSIZE);
    int cannyThreshold = cannyThresholdInitialValue;
    int accumulatorThreshold = accumulatorThresholdInitialValue;
    
    createTrackbar(cannyTrackbarName, windowName, &cannyThreshold, maxCannyThreshold);
    createTrackbar(accumulatorTrackbarName, windowName, &accumulatorThreshold, maxAccumulatorThreshold);
    
    int step = 0;
    while (true)
    {
        bool ret = cap.read(frame);
        if (!ret)
        {
            cerr << "Error: Couldn't capture video frame.\n";
            break;
        }

        cvtColor(frame, src_gray, COLOR_BGR2GRAY);
        GaussianBlur(src_gray, src_gray, Size(9, 9), 2, 2);

        cannyThreshold = max(cannyThreshold, 1);
        accumulatorThreshold = max(accumulatorThreshold, 1);
        step += 1;
        HoughDetection(src_gray, frame, cannyThreshold, accumulatorThreshold);
        if (step % 100 == 0){
            cout << "Canny: " << cannyThreshold << " Accumulator: " << accumulatorThreshold << endl;

        }
        char key = (char)waitKey(1);
    }

    destroyAllWindows();
}
