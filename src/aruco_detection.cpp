#include "aruco_samples_utility.hpp"
#include "camera_info.hpp"
#include "interfacing.hpp"
#include <fstream>

using namespace std;
using namespace cv;

struct Quaternion {
    double x;
    double y;
    double z;
    double w;

    Quaternion(double _x = 0.0, double _y = 0.0, double _z = 0.0, double _w = 1.0)
        : x(_x), y(_y), z(_z), w(_w) {}

    Quaternion& operator*=(double scalar)
    {
        x *= scalar;
        y *= scalar;
        z *= scalar;
        w *= scalar;
        return *this;
    }

    friend ostream& operator<<(ostream& os, const Quaternion& q) 
    {
        os << "[ " << q.x << ", " << q.y << ", " << q.z << ", " << q.w << " ]";
        return os;
    }

    void normalize()
    {
        double norm = sqrt(x * x + y * y + z * z + w * w);
        if (norm > 0.0)
        {
            x /= norm;
            y /= norm;
            z /= norm;
            w /= norm;
        }
        else
        {
            // Handle zero-norm quaternion if necessary
            x = y = z = 0.0;
            w = 1.0;
        }
    }
};

struct Euler {
    double yaw;
    double pitch;
    double roll;


    friend ostream& operator<<(ostream& os, const Euler& e)
    {
        os << "[" << e.roll << ", " << e.pitch << ", " << e.yaw << "]";
        return os;
    }
};

const int QUAT_SIZE = 4;

Quaternion rot2quat(const Mat& R)
{
    // A faster way to compute quaternions, by a game developer from 
    const double m00 = R.at<double>(0, 0);
    const double m01 = R.at<double>(0, 1);
    const double m02 = R.at<double>(0, 2);
    const double m10 = R.at<double>(1, 0);
    const double m11 = R.at<double>(1, 1);
    const double m12 = R.at<double>(1, 2);
    const double m20 = R.at<double>(2, 0);
    const double m21 = R.at<double>(2, 1);
    const double m22 = R.at<double>(2, 2);

    Quaternion q;
    double t;
    if (m22 < 0)
    {
        if (m00 > m11)
        {
            t = 1 + m00 - m11 - m22; 
            q = Quaternion(t, m01+m10, m20+m02, m12-m21);
        }
        else
        {
            t = 1 - m00 - m11 + m22;
            q= Quaternion(m01+m10, t, m12+m21, m20+m02);
        }
    }
    else
    {
        if (m00 < -m11)
        {
            t = 1 - m00 - m11 + m22;
            q= Quaternion(m20+m02, m12+m21, t, m01-m10);
        }
        else
        {
            t = 1 + m00 + m11 + m22;
            q=Quaternion(m12-m21, m20-m02, m01-m10, t);
        }
    }

    q *= 0.5 / sqrt(t);

    return q;
}

double sanity_clip(double angle_rads){
    const double threshold = M_PI / 2.0;
    const double diff = threshold * 2.0;
    if (angle_rads < -threshold){
        angle_rads += diff;
    }
    else if (angle_rads > threshold){
        angle_rads -= diff;
    }
    return angle_rads;
}

Euler quat2euler(const Quaternion& q)
{
    Euler euler_angles;

    // normalize the quaternion to ensure it's a unit quaternion
    double norm = sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    if (norm == 0.0)
    {
        cerr << "zero-norm quaternion cannot be converted" << endl;
        euler_angles.roll = 0.0;
        euler_angles.pitch = 0.0;
        euler_angles.yaw = 0.0;
        return euler_angles;
    }

    double qw = q.w / norm;
    double qx = q.x / norm;
    double qy = q.y / norm;
    double qz = q.z / norm;

    double sinr_cosp = 2.0 * (qw * qx + qy * qz);
    double cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy);
    euler_angles.roll = sanity_clip(atan2(sinr_cosp, cosr_cosp));

    double sinp = sqrt(1 + 2 * (q.w * q.y - q.x * q.z));
    double cosp = sqrt(1 - 2 * (q.w * q.y - q.x * q.z));
    euler_angles.yaw = sanity_clip(2 * atan2(sinp, cosp) - M_PI / 2);

    double siny_cosp = 2.0 * (qw * qz + qx * qy);
    double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
    euler_angles.pitch = atan2(siny_cosp, cosy_cosp);

    return euler_angles;
};



double avg_non_nan(double n1, double n2)
{
    if (isnan(n1)) return n2;
    if (isnan(n2)) return n1;
    return (n1 + n2) / 2.0;
}

array<double, 3> compute_normal_vector(Euler& e)
{
    double n1_x = sin(e.yaw) * cos(e.roll);
    double n1_y = sin(e.yaw) * sin(e.roll);
    double n1_z = cos(e.yaw);
    // cout << n1_x << ", " << n1_y << ", " << n1_z <<endl;

    double n2_x = sin(e.yaw) * cos(e.pitch);
    double n2_y = sin(e.yaw) * sin(e.pitch);
    double n2_z = cos(e.yaw);
    // cout << n2_x << ", " << n2_y << ", " << n2_z <<endl;

    double n_x = avg_non_nan(n1_x, n2_x);
    double n_y = avg_non_nan(n1_y, n2_y);
    double n_z = avg_non_nan(n1_z, n2_z);
    // cout << n_x << ", " << n_y << ", " << n_z <<endl;   

    return array<double, 3> {n_x, n_y, n_z}; 
}

void write_to_kinematics_infile(string line)
{
    ofstream outfile;
    outfile.open(KINEMATICS_INFILE_NAME, std::ios_base::app);
    outfile << line <<endl;
}


void detectMarker(){
    const int camId = 0;
    const bool showRejected = false;
    const bool estimatePose = true; // note: requires loading a calibration file specific to each camera

    aruco::DetectorParameters detectorParams;

    const int CORNER_REFINE_SUBPIX = 1;
    detectorParams.cornerRefinementMethod = CORNER_REFINE_SUBPIX;

    const aruco::Dictionary dictionary = aruco::getPredefinedDictionary(ARUCOTAG_DICTIONARY);

    Mat camMatrix, distCoeffs;
    if(estimatePose) {
        // You can read camera parameters from tutorial_camera_params.yml
        bool readOk = readCameraParameters(CAMERA_PARAMS_FILE, camMatrix, distCoeffs);
        if(!readOk) {
            throw runtime_error("Invalid camera file\n");
        }
    }
    aruco::ArucoDetector detector(dictionary, detectorParams);
    VideoCapture inputVideo;
    int waitTime;
    inputVideo.open(camId);

    double totalTime = 0;
    int count = 0;

    // set coordinate system
    Mat objPoints(4, 1, CV_32FC3);
    objPoints.ptr<Vec3f>(0)[0] = Vec3f(-MARKER_LENGTH_METERS/2.f, MARKER_LENGTH_METERS/2.f, 0);
    objPoints.ptr<Vec3f>(0)[1] = Vec3f(MARKER_LENGTH_METERS/2.f, MARKER_LENGTH_METERS/2.f, 0);
    objPoints.ptr<Vec3f>(0)[2] = Vec3f(MARKER_LENGTH_METERS/2.f, -MARKER_LENGTH_METERS/2.f, 0);
    objPoints.ptr<Vec3f>(0)[3] = Vec3f(-MARKER_LENGTH_METERS/2.f, -MARKER_LENGTH_METERS/2.f, 0);

    while(inputVideo.grab()) {
        Mat image, imageCopy;
        inputVideo.retrieve(image);

        double tick = (double)getTickCount();

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
        double currentTime = ((double)getTickCount() - tick) / getTickFrequency();
        totalTime += currentTime;
        count += 1;
     
        // draw results
        image.copyTo(imageCopy);
        if(!ids.empty()) {
            aruco::drawDetectedMarkers(imageCopy, corners, ids);

            if(estimatePose) {
                for(unsigned int i = 0; i < ids.size(); i++) {
                    drawFrameAxes(imageCopy, camMatrix, distCoeffs, rvecs[i], tvecs[i], MARKER_LENGTH_METERS * 1.5f, 2);
                    if (ids[i] == MARKER_NUMBER && count % 10 == 0){
                        Mat rotation_mat;
                        Rodrigues(rvecs[i], rotation_mat);
                        
                        const double target_z = tvecs[i][2];

                        Quaternion quat = rot2quat(rotation_mat);
                        Euler euler_angles = quat2euler(quat);

                        array<double, 3> normal_vector = compute_normal_vector(euler_angles);

                        cout<<"x: " << normal_vector[0] << " y: " << normal_vector[1] << "z: " << normal_vector[2] << " Time: " << currentTime * 1000 <<"ms" <<endl;

                        const string out = to_string(normal_vector[0]) + "," + to_string(normal_vector[1]) + "," + to_string(normal_vector[2]) + "," + to_string(target_z);
                        write_to_kinematics_infile(out);
                    }


                }
            }
        }

        if(showRejected && !rejected.empty())
            aruco::drawDetectedMarkers(imageCopy, rejected, noArray(), Scalar(100, 0, 255));

        imshow("out", imageCopy);
        char key = (char)waitKey(10);
    }
}

int main(){
    cout << "Starting pose estimation"<<endl;
    detectMarker();
}
