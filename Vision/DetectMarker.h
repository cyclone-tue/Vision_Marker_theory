#ifndef MARKER_VISION_DETECTMARKER_H
#define MARKER_VISION_DETECTMARKER_H

#include <iostream>
#include <opencv/cv.hpp>
#include <cmath>
#include <math.h>
#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include <opencv2/aruco.hpp>        //
#include <opencv2/highgui.hpp>      //
#include "CircleBoard.h"

#define CAMERA_WIDTH 640
#define CAMERA_HEIGHT 480

using Eigen::MatrixXd;
using Eigen::Matrix3d;
using Eigen::VectorXd;

using namespace cv;
using namespace std;
using namespace Eigen;



class vision {
public:
    static bool run(VectorXd& currentState, Vector3d& hoopTransVec, Matrix3d& hoopRotMat);
    static void setupVariables(int camera, const char *calibrationFile);
    static void cleanup();
    static void projectPointsOntoCam(vector<Point3d> cvPoints, VectorXd& currentState, vector<Point2d>& imagePoints);
    static Mat debugFrame;  // for visualization.
private:
    static bool readCameraParameters(String filename, OutputArray cameraMatrix, OutputArray distCoefficients);
    static Matrix3d anglesToRotMatXYZ(double roll, double pitch, double yaw);
    static Matrix3d anglesToRotMatZYX(double roll, double pitch, double yaw);
};



#endif //MARKER_VISION_DETECTMARKER_H
