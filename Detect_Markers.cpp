#include "opencv2/aruco.hpp"
#include "opencv2/highgui.hpp"
#include <iostream>
#include <opencv/cv.hpp>
#include <cmath>
#include <math.h>
#include <netdb.h>
#include "DebugServer.h"




using namespace cv;
using namespace std;

namespace {
    const char* keys =
            "{cal |    | File to load calibration data from}"
            "{cam | 0  | camera input to use}";
    const double markerSize = 0.07; // Marker side length in meters
    const int markerId = 23;
    const double d_after = 1;
    const double v_after = 1;
    const double d_before = 2;
    const double v_in = 1;
    const int debugPort = 8080;

}

static bool readCameraParameters(String filename, OutputArray cameraMatrix, OutputArray distCoefficients){
    FileStorage fs(filename, FileStorage::READ);
    if(!fs.isOpened()){
        fs.release();
        return false;
    }

    Mat localCamera, localDist;
    fs["camera_matrix"] >> localCamera;
    fs["distortion_coefficients"] >> localDist;

    localCamera.copyTo(cameraMatrix);
    localDist.copyTo(distCoefficients);

    //cout << cameraMatrix << endl;
    //cout << distCoefficients << endl;
    fs.release();
    return true;
}
void Dimention3(Mat init, Mat beforeHoop, Mat afterHoop, Vec3d hoop_pos, OutputArray r) {
}


void runPathPlanner(InputArray hoopTransVec, InputArray hoopRotMat, InputOutputArray output, InputArray img){
    Mat init = Mat::zeros(3, 1, CV_64FC1);
    Mat R = Mat::zeros(3, 3, CV_64FC1);
    Mat dist_corr_in = Mat::zeros(3, 1, CV_64FC1);
    Mat vel_corr_in = Mat::zeros(3, 1, CV_64FC1);
    Mat dist_corr_fin = Mat::zeros(3, 1, CV_64FC1);
    Mat vel_corr_fin = Mat::zeros(3, 1, CV_64FC1);

    Mat distanceBeforeHoop = Mat::zeros(3, 1, CV_64FC1);
    Mat velocityBeforeHoop = Mat::zeros(3, 1, CV_64FC1);
    Mat distanceAfterHoop = Mat::zeros(3, 1, CV_64FC1);
    Mat velocityAfterHoop = Mat::zeros(3, 1, CV_64FC1);
    Vec3d hoop_pos;
    hoopTransVec.copyTo(hoop_pos);

    distanceAfterHoop.at<double>(1,0) = d_after;
    velocityAfterHoop.at<double>(1,0) = v_after;
    distanceBeforeHoop.at<double>(1,0) = d_before;
    velocityBeforeHoop.at<double>(1,0) = v_in;

    dist_corr_in = R * distanceBeforeHoop;
    dist_corr_fin = R * distanceAfterHoop;
    vel_corr_in = R * velocityBeforeHoop;
    vel_corr_fin = R * velocityAfterHoop;

    Mat pointBeforeHoop = hoop_pos - dist_corr_in;
    Mat beforeHoop = Mat::zeros(2, 3, CV_64F);
    beforeHoop.col(0) = pointBeforeHoop;
    beforeHoop.col(1) = vel_corr_in;

    Mat pointFinal = Mat::zeros(1,3, CV_64F);
    pointFinal = hoop_pos + dist_corr_fin;
    Mat afterHoop = Mat::zeros(3, 3, CV_64F);
    afterHoop.col(0) = pointFinal;
    afterHoop.col(1) = vel_corr_fin;

    Mat r;
    Dimention3(init, beforeHoop, afterHoop, hoop_pos, r);
    r.copyTo(output);

}



int main(int argc, char* argv[]){
    CommandLineParser parser = CommandLineParser(argc, argv, keys);
    parser.about("Run marker detection code.");

    if(argc < 2){
        parser.printMessage();
        return 0;
    }

    int cam = parser.get<int>("cam");
    String filename = parser.get<String>("cal");
    VideoCapture cap;
    cap.open(cam);

    //DebugServer server = DebugServer("127.0.0.1", debugPort);
    /*DebugServer debugServer = DebugServer("127.0.0.1", debugPort);
    struct addrinfo * addressinfo = debugServer.getAddressInfo();*/

    Mat img;
    cap >> img;
    double fps = cap.get(CV_CAP_PROP_FPS);
    int codec = VideoWriter::fourcc('M', 'J', 'E' ,'G');

    VideoWriter server = VideoWriter("udp://127.0.0.1:8080", codec, fps, img.size(), true);

    Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(aruco::DICT_ARUCO_ORIGINAL);

    Mat cameraMatrix, distCoef;

    if(!readCameraParameters(filename, cameraMatrix, distCoef)){
        cout << "Could not load camera calibration file: " << filename << endl;
    }else{
        cout << cameraMatrix << endl;
        cout << distCoef << endl;
    }

    while(cap.grab()){
        Mat image, imageCopy;
        cap.retrieve(image);
        image.copyTo(imageCopy);

        vector<int> ids;
        vector<vector<Point2f>> corners;
        aruco::detectMarkers(image, dictionary, corners, ids);

        //At least one marker detected
        if(!ids.empty()){
            aruco::drawDetectedMarkers(imageCopy, corners, ids);
            vector<Vec3d> rvecs, tvecs;
            aruco::estimatePoseSingleMarkers(corners, markerSize, cameraMatrix, distCoef, rvecs, tvecs);
            for(int i=0; i<ids.size(); i++){
                if(ids[i] == markerId){
                    Vec3d offset = Vec3d(0,-0.37,0);

                    Mat rotMat;

                    Rodrigues(rvecs[i], rotMat);

                    Mat camOffset = rotMat*Mat(offset);
                    Vec3d pixelOffset;

                    camOffset.col(0).copyTo(pixelOffset);
                    tvecs[i] += pixelOffset;
                    double sy = sqrt(pow(rotMat.at<double>(0,0),2) + pow(rotMat.at<double>(1,0), 2));
                    bool singular = sy < 1e-6;
                    double rollTemp, pitchTemp, yawTemp;
                    if(!singular){
                        rollTemp = atan2(rotMat.at<double>(2,1), rotMat.at<double>(2,2));
                        pitchTemp = atan2(-rotMat.at<double>(2,0), sy);
                        yawTemp = atan2(rotMat.at<double>(1,0), rotMat.at<double>(0,0));
                    }else{
                        rollTemp = atan2(rotMat.at<double>(2,1), rotMat.at<double>(2,2));
                        pitchTemp = 0;
                        yawTemp = atan2(rotMat.at<double>(1,0), rotMat.at<double>(0,0));
                    }

                    double yaw = -pitchTemp;
                    double roll = -yawTemp;
                    double pitch = M_PI - rollTemp;
                    if(pitch > M_PI){
                        pitch -= 2*M_PI;
                    }

                    double x = tvecs[i][0];
                    double y = tvecs[i][1];
                    double z = tvecs[i][2];



                    //double dist = rotMatInversed.dot(tvecs[0] * -1);
                    //cout << "Roll: " << rollTemp << endl;
                    //cout << "Pitch: " << pitchTemp << endl;
                    //cout << "Yaw: "  << yawTemp << endl;
                    cout << "x " << x << ", y : " << y << ", z :" << z <<  ", yaw: " << yaw/M_PI*180 << ", pitch: " << pitch/M_PI*180 << ", roll: " << roll/M_PI*180 << endl;
                    aruco::drawAxis(imageCopy, cameraMatrix, distCoef, rvecs[i],tvecs[i], markerSize);
                    Mat path;
                    runPathPlanner(tvecs[i], rotMat, path, imageCopy);
                }

            }

        }
        imshow("out", imageCopy);
        //server.setImage(imageCopy);
        Mat outputImage;
        cvtColor(imageCopy, outputImage, COLOR_Lab2BGR);
        server.write(imageCopy);
        char key = (char) waitKey(1);
        if (key == 27) break;

    }

    cap.release();
    //server.stop();
    server.release();

}

