#include "opencv2/aruco.hpp"
#include "opencv2/highgui.hpp"
#include <iostream>

using namespace cv;
using namespace std;

namespace {
    const char* keys =
            "{cal |    | File to load calibration data from}";
    const double markerSize = 0.07; // Marker side length in meters
}

static bool readCameraParameters(String filename, OutputArray cameraMatrix, OutputArray distCoefficients){
    FileStorage fs(filename, FileStorage::READ);
    if(!fs.isOpened()){
        return false;
    }

    Mat localCamera, localDist;
    fs["camera_matrix"] >> localCamera;
    fs["distortion_coefficients"] >> localDist;

    localCamera.copyTo(cameraMatrix);
    localDist.copyTo(distCoefficients);

    //cout << cameraMatrix << endl;
    //cout << distCoefficients << endl;
    return true;
}


int main(int argc, char* argv[]){
    CommandLineParser parser = CommandLineParser(argc, argv, keys);
    parser.about("Run marker detection code.");

    if(argc < 1){
        parser.printMessage();
        return 0;
    }
    String filename = parser.get<String>("cal");
    VideoCapture cap;
    cap.open(0);

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

        }
        imshow("out", imageCopy);
        char key = (char) waitKey(10);
        if (key == 27) break;

    }

}

