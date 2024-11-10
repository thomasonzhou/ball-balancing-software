#include "aruco_samples_utility.hpp"
#include "camera_info.hpp"

using namespace std;
using namespace cv;

cv::Mat rot2euler(const cv::Mat & rotationMatrix)
{
    cv::Mat euler(3,1,CV_64F);

    double m00 = rotationMatrix.at<double>(0,0);
    double m02 = rotationMatrix.at<double>(0,2);
    double m10 = rotationMatrix.at<double>(1,0);
    double m11 = rotationMatrix.at<double>(1,1);
    double m12 = rotationMatrix.at<double>(1,2);
    double m20 = rotationMatrix.at<double>(2,0);
    double m22 = rotationMatrix.at<double>(2,2);

    double bank, attitude, heading;

    // Assuming the angles are in radians.
    if (m10 > 0.998) { // singularity at north pole
        bank = 0;
        attitude = CV_PI/2;
        heading = atan2(m02,m22);
    }
    else if (m10 < -0.998) { // singularity at south pole
        bank = 0;
        attitude = -CV_PI/2;
        heading = atan2(m02,m22);
    }
    else
    {
        bank = atan2(-m12,m11);
        attitude = asin(m10);
        heading = atan2(-m20,m00);
    }

    euler.at<double>(0) = bank;
    euler.at<double>(1) = attitude;
    euler.at<double>(2) = heading;

    return euler;
}

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
    int count = 0;

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
        count += 1;
     
        //! [aruco_draw_pose_estimation]
        // draw results
        image.copyTo(imageCopy);
        if(!ids.empty()) {
            cv::aruco::drawDetectedMarkers(imageCopy, corners, ids);

            if(estimatePose) {
                for(unsigned int i = 0; i < ids.size(); i++) {
                    cv::drawFrameAxes(imageCopy, camMatrix, distCoeffs, rvecs[i], tvecs[i], MARKER_LENGTH_METERS * 1.5f, 2);
                    if (ids[i] == MARKER_NUMBER && count % 50 == 0){
                        cv::Mat rotation_mat;
                        cv::Rodrigues(rvecs[i], rotation_mat);
                        cv::Mat euler_vec = rot2euler(rotation_mat);
                        
                        const double target_x = -euler_vec.at<double>(0);
                        const double target_y = euler_vec.at<double>(1);
                        const double t_z = euler_vec.at<double>(2);
                        const double angle = sqrt(target_x * target_x + target_y * target_y);

                        cout << "x: " << target_x 
                        << " y: " << target_y 
                        <<"z: " << t_z
                        << " angle: " << angle <<endl;



                        // cout<<"Detection Time: " << currentTime * 1000 <<"ms"
                        // << "\nMarker found: " << ids[i] 
                        // << "\nrvec: " << rvecs[i] 
                        // << "\nrotation: " << rotation_mat
                        // << "\neuler: " <<euler_vec
                        // << "\ntvec: " << tvecs[i] << endl;
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
