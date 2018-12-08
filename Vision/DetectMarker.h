#ifndef MARKER_VISION_DETECTMARKER_H
#define MARKER_VISION_DETECTMARKER_H

#include <iostream>
#include <opencv/cv.hpp>
#include <cmath>
#include <math.h>
#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include "opencv2/aruco.hpp"
#include "opencv2/highgui.hpp"
#include "CircleBoard.h"


using Eigen::MatrixXd;
using Eigen::Matrix3d;
using Eigen::VectorXd;

using namespace cv;
using namespace std;
using namespace Eigen;

class vision {
public:
    static bool readCameraParameters(String filename, OutputArray cameraMatrix, OutputArray distCoefficients);
    static bool run(Vector3d *hoopTransVec, Matrix3d *hoopRotMat);
    static void setupVariables(int camera, const char *calibrationFile);
};





#endif //MARKER_VISION_DETECTMARKER_H
