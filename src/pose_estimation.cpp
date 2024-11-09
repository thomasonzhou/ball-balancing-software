#include "aruco_samples_utility.hpp"
#include "camera_info.hpp"

using namespace std;
using namespace cv;

void detectMarker(){
    const int camId = 0;
    const bool showRejected = false;
    const bool estimatePose = true; // note: requires loading a calibration file specific to each camera

    cv::aruco::DetectorParameters detectorParams;

    const int CORNER_REFINE_SUBPIX = 1;
    detectorParams.cornerRefinementMethod = CORNER_REFINE_SUBPIX;

    const cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(ARUCOTAG_DICTIONARY);

    //! [aruco_pose_estimation1]
    Mat camMatrix, distCoeffs;
    if(estimatePose) {
        // You can read camera parameters from tutorial_camera_params.yml
        bool readOk = readCameraParameters(CAMERA_PARAMS_FILE, camMatrix, distCoeffs);
        if(!readOk) {
            throw std::runtime_error("Invalid camera file\n");
        }
    }
    //! [aruco_pose_estimation1]
    //! [aruco_detect_markers]
    cv::aruco::ArucoDetector detector(dictionary, detectorParams);
    cv::VideoCapture inputVideo;
    int waitTime;
    inputVideo.open(camId);
    waitTime = 10;

    double totalTime = 0;

    //! [aruco_pose_estimation2]
    // set coordinate system
    cv::Mat objPoints(4, 1, CV_32FC3);
    objPoints.ptr<Vec3f>(0)[0] = Vec3f(-MARKER_LENGTH_METERS/2.f, MARKER_LENGTH_METERS/2.f, 0);
    objPoints.ptr<Vec3f>(0)[1] = Vec3f(MARKER_LENGTH_METERS/2.f, MARKER_LENGTH_METERS/2.f, 0);
    objPoints.ptr<Vec3f>(0)[2] = Vec3f(MARKER_LENGTH_METERS/2.f, -MARKER_LENGTH_METERS/2.f, 0);
    objPoints.ptr<Vec3f>(0)[3] = Vec3f(-MARKER_LENGTH_METERS/2.f, -MARKER_LENGTH_METERS/2.f, 0);
    //! [aruco_pose_estimation2]

    while(inputVideo.grab()) {
        cv::Mat image, imageCopy;
        inputVideo.retrieve(image);

        double tick = (double)getTickCount();

        //! [aruco_pose_estimation3]
        vector<int> ids;
        vector<vector<Point2f> > corners, rejected;

        // detect markers and estimate pose
        detector.detectMarkers(image, corners, ids, rejected);

        size_t nMarkers = corners.size();
        vector<Vec3d> rvecs(nMarkers), tvecs(nMarkers);

        if(estimatePose && !ids.empty()) {
            // Calculate pose for each marker
            for (size_t i = 0; i < nMarkers; i++) {
                solvePnP(objPoints, corners.at(i), camMatrix, distCoeffs, rvecs.at(i), tvecs.at(i));
            }
        }
        //! [aruco_pose_estimation3]
        double currentTime = ((double)getTickCount() - tick) / getTickFrequency();
        totalTime += currentTime;
     
        //! [aruco_draw_pose_estimation]
        // draw results
        image.copyTo(imageCopy);
        if(!ids.empty()) {
            cv::aruco::drawDetectedMarkers(imageCopy, corners, ids);

            if(estimatePose) {
                for(unsigned int i = 0; i < ids.size(); i++) {
                    cv::drawFrameAxes(imageCopy, camMatrix, distCoeffs, rvecs[i], tvecs[i], MARKER_LENGTH_METERS * 1.5f, 2);
                    if (ids[i] == MARKER_NUMBER){
                        cout<<"Detection Time: " << currentTime * 1000 <<"ms"
                        << "\nMarker found: " << ids[i] 
                        << "\nrvec: " << rvecs[i] 
                        << "\ntvec: " << tvecs[i] << endl;
                    }
                }
            }
        }
        //! [aruco_draw_pose_estimation]

        if(showRejected && !rejected.empty())
            cv::aruco::drawDetectedMarkers(imageCopy, rejected, noArray(), Scalar(100, 0, 255));

        imshow("out", imageCopy);
        char key = (char)waitKey(waitTime);
        if(key == 27) break;
    }
    //! [aruco_detect_markers]
}

int main(){
    std::cout << "Starting pose estimation"<<std::endl;
    detectMarker();
}
