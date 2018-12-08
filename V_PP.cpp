
#include "Vision/vision.h"
#include "Path_Planning/path_planner.h"

#include <iostream>
#include <cmath>
#include <math.h>
#include <Eigen/Dense>

//PATH_PLANNER()
//VISION()

using Eigen::MatrixXd;
using Eigen::Matrix3d;
using Eigen::VectorXd;
using Eigen::DiagonalMatrix;

using namespace std;
using namespace Eigen;

const double g = 9.81;


bool runFrame(bool visualize, VectorXd currentState, VectorXd currentTorque, MatrixXd path, VectorXd timeDiffs, MatrixXd torques){
    bool foundPath;

    Vector3d hoopTransVec;
    Matrix3d hoopRotMat;

    bool foundHoop = vision::run(hoopTransVec, hoopRotMat);
    if(foundHoop == true){
        foundPath = path_planner::run(currentState, currentTorque, hoopTransVec, hoopRotMat, path, timeDiffs, torques);
    }

    return foundPath;
}


void setup(){}

double* output_to_py(VectorXd currentState, VectorXd currentTorque, bool* pathLength, bool visualize){
    double* db_p;  // stands for ...
    MatrixXd path;
    VectorXd timeDiffs;
    MatrixXd torques;

    *pathLength = runFrame(visualize, currentState, currentTorque, path, timeDiffs, torques);

    MatrixXd outputInfo(*pathLength, 12+1+4);
    outputInfo << path, 0, timeDiffs, torques;

    // copy path to output array
    double db_array[*pathLength][12 + 1 + 4];
    Map<MatrixXd>(&db_array[0][0], path.rows(), path.cols()) = outputInfo;
    db_p = &db_array[0][0];

    return db_p;
}

int main(){

    vision::setupVariables(0, "Vision/drone_calibration.txt");

    VectorXd currentState(12);
    VectorXd currentTorque(4);
    bool* pathLength;
    bool visualize;
    currentState << 0,0,0, 0,0,0, 0,0,0, 0,0,0;
    currentTorque << 0,0,0,0;

    double *db_p;
    db_p = output_to_py(currentState, currentTorque, pathLength, visualize);

    return 0;
}


