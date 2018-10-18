#include "opencv2/aruco.hpp"
#include "opencv2/highgui.hpp"
#include <iostream>
#include <opencv/cv.hpp>			//contains CommandLineParser, aruco
#include <cmath>
#include <math.h>
//#include <Eigen/Dense>
//#include <opencv2/core/eigen.hpp>

//using Eigen::MatrixXd;
//using Eigen::Matrix3d;
//using Eigen::VectorXd;
//using Eigen::DiagonalMatrix;

using namespace cv;
using namespace std;
//using namespace Eigen;

VectorXd coef_vec2(6);

namespace {
    const char* keys =
            "{cal |    | File to load calibration data from}"
            "{cam | 0  | camera input to use}";
    const double markerSize = 0.07; // Marker side length in meters
    const int markerId = 23;
    const Vec3d hoop_offset = Vec3d(0,0,0);//Offset variable in world space from center of marker to center of hoop.
}






// load cameraMatrix and distCoefficients from file
static bool readCameraParameters(String filename, OutputArray cameraMatrix, OutputArray distCoefficients){
    FileStorage fs(filename, FileStorage::READ);	//open file
    if(!fs.isOpened()){		//return if not opened correctly
        cout << "Could not load camera calibration file: " << filename << endl;
		fs.release();
        return false;
    }
	
	OutputArray cameraMatrix, distCoefficients;
    fs["camera_matrix"] >> cameraMatrix;
    fs["distortion_coefficients"] >> distCoefficients;
    fs.release();
	
    return true;
}

VideoCapture getCameraVar(parser){
	// get camera

    int cam = parser.get<int>("cam");				//camera
    VideoCapture cap;								//image capture variable
    cap.open(cam);									//link camera to capture
	
	return cap
}


// argument count, argument vector
int main(int argc, char* argv[]){
	
	//parser stuff
    CommandLineParser parser = CommandLineParser(argc, argv, keys);
    //parser.about("Run marker detection code.");
    if(argc < 2){					//check number of arguments
        parser.printMessage();		//print some standard information of parser (its arguments)
        return 0;
    }
	
	
	VideoCapture cap = getCameraVar(parser)

    
    Mat cameraMatrix, distCoef;		// some matrices , distortion coefficients
	String filename = parser.get<String>("cal");	//filename for calibration values
	getCameraParameters(filename, cameraMatrix, distCoef); 
	
	Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(aruco::DICT_ARUCO_ORIGINAL);		//make dictionary for what??

	
	
	
	
	

    while(cap.grab()){
        Mat image, imageCopy;
		
        cap.retrieve(image);
        //image.copyTo(imageCopy);	//why the hell

        vector<int> ids;						// ID's of detected markers
        vector<vector<Point2f>> corners;		// corners of markers
        aruco::detectMarkers(image, dictionary, corners, ids);		//write to corners, ids

        //At least one marker detected
        if(!ids.empty()){
            aruco::drawDetectedMarkers(image, corners, ids);		//draw marker
            vector<Vec3d> rvecs, tvecs;		// rotation and translation of marker in body frame
            aruco::estimatePoseSingleMarkers(corners, markerSize, cameraMatrix, distCoef, rvecs, tvecs);
			
            for(int i=0; i<ids.size(); i++){
                if(ids[i] == markerId){
                    
					Mat rotMat;								// body to world frame
                    Rodrigues(rvecs[i], rotMat);			// Calculate rotation matrix for marker

                    Mat pos = rotMat.t()*Mat(tvecs[i]); 	// Calculate marker position in world space
                    pos = pos + Mat(hoop_offset); 			// Add offset in world space to get center of hoop.
                    

                    Mat pixelsTranslated = rotMat * pos;	// what does this do? you rotate the marker position in world space with rotMat?
                    Vec3d pixels;
                    pixelsTranslated.copyTo(pixels);
                    tvecs[i] = pixels;
					
                    double sy = sqrt(pow(rotMat.at<double>(0,0), 2) + pow(rotMat.at<double>(1,0), 2));		//what is sy? sin(theta)?
                    bool singular = sy < 1e-6;
					
                    double rollTemp, pitchTemp, yawTemp;		//temp is what
                    if(!singular){
                        rollTemp = atan2(rotMat.at<double>(2,1), rotMat.at<double>(2,2));
                        pitchTemp = atan2(-rotMat.at<double>(2,0), sy);
                        yawTemp = atan2(rotMat.at<double>(1,0), rotMat.at<double>(0,0));
                    }else{
                        rollTemp = atan2(rotMat.at<double>(2,1), rotMat.at<double>(2,2));
                        pitchTemp = 0;
                        yawTemp = atan2(rotMat.at<double>(1,0), rotMat.at<double>(0,0));
                    }

                    double yaw = -pitchTemp;		//thats not nice
                    double roll = -yawTemp;
                    double pitch = M_PI - rollTemp;
                    if(pitch > M_PI){
                        pitch -= 2*M_PI;
                    }

                    double x = pos.at<double>(0,0);
                    double y = pos.at<double>(1,0);
                    double z = pos.at<double>(2,0);



                    //cout << "x " << x << ", y : " << y << ", z :" << z <<  ", yaw: " << yaw/M_PI*180 << ", pitch: " << pitch/M_PI*180 << ", roll: " << roll/M_PI*180 << endl;
                    aruco::drawAxis(imageCopy, cameraMatrix, distCoef, rvecs[i],tvecs[i], markerSize);
                    Mat path;
                    runPathPlanner(pos, rotMat, path);

                    vector<Point3d> pathPoints;
                    for(int i = 0; i < path.rows; i+=10){
                        Mat row = path.row(i);
                        pathPoints.push_back(Point3d(row.at<double>(0,0), row.at<double>(1,0), row.at<double>(2,0)));
                    }
                    //cout << "Created path points." << endl;
                    vector<Point2d> imagePoints;
                    projectPoints(pathPoints, Vec3d(0,0,0), Vec3d(0,0,0), cameraMatrix, distCoef, imagePoints);
                    for (int j = 0; j < imagePoints.size() - 1; j++) {
                        line(imageCopy, imagePoints[j], imagePoints[j + 1], Scalar(255/imagePoints.size() * j,0,255/imagePoints.size()*(imagePoints.size() - j)), 3);
                    }
                    //cout << "Drawn lines" << endl;
                    //cout << path << endl;
                }

            }

        }
        imshow("out", imageCopy);
        char key = (char) waitKey(1);
        if (key == 27) break;

    }

    cap.release();

}



















