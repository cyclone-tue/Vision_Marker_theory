

#include "V_PP.h"


int runFrame(bool visualize, VectorXd currentState, VectorXd currentTorque, MatrixXd& path, VectorXd& timeDiffs, MatrixXd& torques){

    int pathLength = 0;

    Vector3d hoopTransVec;
    Matrix3d hoopRotMat;

    bool foundHoop = vision::run(currentState, hoopTransVec, hoopRotMat);


    if(foundHoop == true){
        writeDebug("\nstart path_planner.\n", "log", false);

        pathLength = path_planner::run(currentState, currentTorque, hoopTransVec, hoopRotMat, path, timeDiffs, torques);

    }


    return pathLength;
}


void setup(const char* camera_calibration_file){
    vision::setupVariables(0, camera_calibration_file);
    writeDebug("setup done\n", "log", true);
    return;
}


void cleanup(){
    vision::cleanup();
}

/*
The positive direction of the z-axis of the world frame is parallel to the gravitational acceleration.
All used frames stand still with respect to the earth.

The arguments are the following:
currentState    : state of drone in world frame.
currentTorque   : [thrust, torqueX, torqueY, torqueZ].
*pathLength     : number of states in the returned path.
                : initial value is not used.

return value    : pointer to array of doubles, being the elements([row0,row1, .. row_n]) of the following matrix:
                : The matrix is a concatenation of the following matrices, each with *pathLength rows.
                : path      : each row is a state in world space of the path.
                : timeDiffs : each i'th element is the time difference between the i-1'th and i'th state in path.
                            : the first element is the time difference between the current time and the time of the first state in path.
                : torques   : each row is the [thrust, torqueX, torqueY, torqueZ] corresponding to the same row in path.
*/
double* output_to_py(double* currentStateArray, double* currentTorqueArray, int* pathLength, bool visualize){

    VectorXd currentState(12);
    VectorXd currentTorque(4);
    for(int i = 0; i <= 11; i++){
        currentState(i) = currentStateArray[i];
    }
    for(int i = 0; i <= 3; i++){
        currentTorque(i) = currentTorqueArray[i];
    }

    double* db_p;  // stands for ...
    MatrixXd path(0,12);
    VectorXd timeDiffs(0);
    MatrixXd torques(0,4);

    *pathLength = runFrame(visualize, currentState, currentTorque, path, timeDiffs, torques);

    if(visualize) runVisualize(currentState, path, *pathLength!=0);

    if(*pathLength != 0) {      // should be more robust.

        MatrixXd outputInfo(*pathLength, 12 + 1 + 4);
        outputInfo << path, timeDiffs, torques;

        writeDebug("path,timeDiffs,torques:\n", "log", false);
        writeDebug(outputInfo, "log", false);

        //copy path to output array
        double db_array[*pathLength][12 + 1 + 4];
        Map<MatrixXd>(&db_array[0][0], outputInfo.rows(), outputInfo.cols()) = outputInfo;
        db_p = &db_array[0][0];
        return db_p;
    }
    
    return nullptr;
}

/*
path contains x times 12 elements.
 */
void runVisualize(VectorXd& currentState, MatrixXd& path, bool displayPath){

    Mat frame;
    vision::debugFrame.copyTo(frame);

    if(displayPath) {
        MatrixXd points(path.rows(), path.cols());

        writeDebug(path, "log", false);

        vector<Point3d> cvPoints;
        for (int row = 0; row < points.rows(); row++) {
            cvPoints.push_back(Point3d(path(row, 0), path(row, 1), path(row, 2)));
        }

        vector<Point2d> imagePoints;
        vision::projectPointsOntoCam(cvPoints, currentState, imagePoints);



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

    VectorXd currentState(12);
    currentState << 4,10,7, 0,0,0, 0,0,0, 0,0,0;

    VectorXd currentTorque(4);
    currentTorque << 0,0,0,0;

    Vector3d hoopTransVec;
    hoopTransVec << 10, 0, 0;

    Matrix3d hoopRotMat;
    hoopRotMat << 1,0,0, 0,1,0, 0,0,1;

    MatrixXd path(0,12);
    VectorXd timeDiffs(0);
    MatrixXd torques(0,4);

    int pathLength = path_planner::run(currentState, currentTorque, hoopTransVec, hoopRotMat, path, timeDiffs, torques);

    MatrixXd outputInfo(pathLength, 12 + 1 + 4);
    outputInfo << path, timeDiffs, torques;

    writeDebug("path,timeDiffs,torques:\n", "log", false);
    writeDebug(outputInfo, "log", false);

    return 0;

    /*
    setup("../laptop_calibration.txt");


    double currentState [12] = {0,0,0, 0,0,0, 0,0,0, 0,0,0};
    double currentTorque [4] = {0,0,0,0};
    int* pathLength = new int(1);
    bool visualize = true;


    double *db_p;
    while(true){
        db_p = output_to_py(currentState, currentTorque, pathLength, visualize);
    }

    return 0;*/
}




void writeDebug(String info, String destination, bool display){

    if(display){
        cout << info << endl;
    }

    if(destination == "log") {
        ofstream file;
        file.open ("log.txt",ios::app);
        file << info;
        file.close();
    }

    return;
}

void writeDebug(MatrixXd info, String destination, bool display){

    if(display){
        cout << info << endl;
    }

    if(destination == "log") {
        ofstream file;
        file.open ("log.txt",ios::app);
        file << info << endl;
        file.close();
    }

    return;
}





