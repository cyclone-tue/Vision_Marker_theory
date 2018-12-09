

#include "V_PP.h"


int runFrame(bool visualize, VectorXd currentState, VectorXd currentTorque, MatrixXd& path, VectorXd& timeDiffs, MatrixXd& torques){

    int pathLength = 0;

    Vector3d hoopTransVec;
    Matrix3d hoopRotMat;

    cout << "run vision" << endl;
    bool foundHoop = vision::run(currentState, hoopTransVec, hoopRotMat, visualize);


    if(foundHoop == true){
        cout << "run path planner" << endl;
        pathLength = path_planner::run(currentState, currentTorque, hoopTransVec, hoopRotMat, path, timeDiffs, torques);
    }
    else{
        cout << "    no hoop was found" << endl;
    }

    return pathLength;
}


void setup(){
    vision::setupVariables(0, "../laptop_calibration.txt");
}

void cleanup(){
    vision::cleanup();
}


double* output_to_py(VectorXd currentState, VectorXd currentTorque, int* pathLength, bool visualize){
    double* db_p;  // stands for ...
    MatrixXd path;
    VectorXd timeDiffs;
    MatrixXd torques;

    *pathLength = runFrame(visualize, currentState, currentTorque, path, timeDiffs, torques);

    if(visualize) runVisualize(path, *pathLength!=0);

    if(*pathLength != 0) {      // should be more robust.

        MatrixXd outputInfo(*pathLength, 12 + 1 + 4);
        outputInfo << path, timeDiffs, torques;

        //copy path to output array
        double db_array[*pathLength][12 + 1 + 4];
        Map<MatrixXd>(&db_array[0][0], outputInfo.rows(), outputInfo.cols()) = outputInfo;
        db_p = &db_array[0][0];
        return db_p;
    }

    return nullptr;
}

void runVisualize(MatrixXd& path, bool displayPath){

    Mat frame;
    vision::debugFrame.copyTo(frame);

    if(displayPath) {
        MatrixXd points(path.rows(), path.cols());
        points << path.block(0, 0, path.rows(), 3);           // each row is an [x,y,z] point.

        vector<Point3d> cvPoints;
        for (int row = 0; row < points.rows(); row++) {
            cvPoints.push_back(Point3d(points(row, 0), points(row, 1), points(row, 2)));
        }
        cout << cvPoints << endl;

        vector<Point2d> imagePoints;
        vision::projectPointsOntoCam(cvPoints, imagePoints);


        for (int j = 0; j < imagePoints.size() - 1; j++) {
            line(frame, imagePoints[j], imagePoints[j + 1], Scalar(int(255 / imagePoints.size() * j), 0,
                                                                   int(255 / imagePoints.size() *
                                                                       (imagePoints.size() - j))), 3);
        }

    }

    imshow("out", frame);
    waitKey(1);
    return;
}


int main(){

    setup();
    cout << "setup done" << endl;

    VectorXd currentState(12);
    VectorXd currentTorque(4);
    int* pathLength = new int(1);
    bool visualize = true;
    currentState << 0,0,0, 0,0,0, 0,0,0, 0,0,0;
    currentTorque << 0,0,0,0;

    double *db_p;
    //for(int i = 0; i<=1000; i++) {
    while(true){
        db_p = output_to_py(currentState, currentTorque, pathLength, visualize);
    }

    return 0;
}


