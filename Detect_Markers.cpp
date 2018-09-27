#include "opencv2/aruco.hpp"
#include "opencv2/highgui.hpp"
#include <iostream>
#include <opencv/cv.hpp>
#include <cmath>

using namespace cv;
using namespace std;

namespace {
    const char* keys =
            "{cal |    | File to load calibration data from}"
            "{cam | 0  | camera input to use}";
    const double markerSize = 0.07; // Marker side length in meters
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
                aruco::drawAxis(imageCopy, cameraMatrix, distCoef, rvecs[i],tvecs[i], markerSize);
            }
            Mat rotMat;

            Rodrigues(rvecs[0], rotMat);

            double sy = sqrt(pow(rotMat.at<double>(0,0),2) + pow(rotMat.at<double>(1,0), 2));
            bool singular = sy < 1e-6;
            double roll, pitch, yaw;
            if(!singular){
                roll = atan2(rotMat.at<double>(2,1), rotMat.at<double>(2,2));
                pitch = atan2(-rotMat.at<double>(2,0), sy);
                yaw = atan2(rotMat.at<double>(1,0), rotMat.at<double>(0,0));
            }else{
                roll = atan2(rotMat.at<double>(2,1), rotMat.at<double>(2,2));
                pitch = 0;
                yaw = atan2(rotMat.at<double>(1,0), rotMat.at<double>(0,0));
            }

            double x = tvecs[0][0];
            double y = tvecs[0][1];
            double z = tvecs[0][2];


            //double dist = rotMatInversed.dot(tvecs[0] * -1);
            //cout << "Roll: " << roll << endl;
            //cout << "Pitch: " << pitch << endl;
            //cout << "Yaw: "  << yaw << endl;
            cout << "x " << x << ", y : " << y << ", z :" << z << endl;
        }
        imshow("out", imageCopy);
        char key = (char) waitKey(1);
        if (key == 27) break;

    }

    cap.release();

}

