

#include "V_PP.h"



// ==== functions to be called from python scripts ====


void setup(const char* camera_calibration_file){
    time_t t = time(0);
    string now(ctime(&t));
    writeDebug("\n=========================== new setup() started at " + now + "\n");
    vision::setupVariables(0, camera_calibration_file);
    path_planner::init();
    writeDebug("setup done\n");
    return;
}


void cleanup(){
    vision::cleanup();
    path_planner::release();
    return;
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

    VectorXd currentState(12);      // input variables
    Vector4d currentTorque;
    double* output_ptr;             // output variables
    MatrixXd path(0,12);
    VectorXd timeDiffs(0);
    MatrixXd torques(0,4);

    currentState = arrayToEigen(currentStateArray, 12);
    currentTorque = arrayToEigen(currentTorqueArray, 4);

    bool success = runFrame(currentState, currentTorque, path, timeDiffs, torques);
    if(visualize){
        runVisualize(currentState, path, success);
    }
    if(success) {
        *pathLength = path.rows();
        MatrixXd outputInfo(*pathLength, 12 + 1 + 4);
        outputInfo << path, timeDiffs, torques;

        writeDebug("path,timeDiffs,torques:\n");
        writeDebug(outputInfo);

        //copy path to output array
        double output_array[*pathLength][12 + 1 + 4];
        Map<MatrixXd>(&output_array[0][0], outputInfo.rows(), outputInfo.cols()) = outputInfo;
        output_ptr = &output_array[0][0];
        return output_ptr;
    }
    
    return nullptr;
}


// ==== other functions ====


bool runFrame(VectorXd& currentState, Vector4d& currentTorque, MatrixXd& path, VectorXd& timeDiffs, MatrixXd& torques){

    bool success = false;
    Vector3d hoopTransVec;
    Matrix3d hoopRotMat;

    bool foundHoop = vision::run(currentState, hoopTransVec, hoopRotMat);
    if(foundHoop){
        success = path_planner::run(currentState, currentTorque, hoopTransVec, hoopRotMat, path, timeDiffs, torques);
    }
    return success;
}

VectorXd arrayToEigen(double* array, int length){
    VectorXd vector(length);
    for(int i = 0; i < length; i++){
        vector(i) = array[i];
    }
    return vector;
}


// ==== test scenarios ====


void test_PP(){

    time_t t = time(0);
    string now(ctime(&t));
    writeDebug("\n========================== new test_PP() started at " + now + "\n");

    VectorXd currentState(12);  // input variables
    Vector4d currentTorque;
    Vector3d hoopTransVec;
    Matrix3d hoopRotMat;
    MatrixXd path(0,12);        // output variables
    VectorXd timeDiffs(0);
    MatrixXd torques(0,4);

    currentState << 2,0,0, 0,0,0, 0,0,0, 0,0,0;         // input to the path planner
    currentTorque << 20,0,0,0;
    hoopTransVec << 10, 0, 0;
    hoopRotMat << 1,0,0, 0,1,0, 0,0,1;

    bool success = path_planner::run(currentState, currentTorque, hoopTransVec, hoopRotMat, path, timeDiffs, torques);

    MatrixXd outputInfo(path.rows(), 12+1+4);
    outputInfo << path, timeDiffs, torques;

    writeDebug("outputInfo:\n");
    writeDebug(outputInfo);

    if(not success){
        writeDebug("Test was unsuccessful...\n");
    }

    writeDebug("End of test.\n");
    return;
}


void test_V_PP(){
    setup("../laptop_calibration.txt");

    double currentState [12] = {0,0,0, 0,0,0, 0,0,0, 0,0,0};
    double currentTorque [4] = {0,0,0,0};
    int* pathLength = new int(1);
    bool visualize = true;

    double *output_ptr;   // contains path, timeDiffs, torques
    while(true){
        output_ptr = output_to_py(currentState, currentTorque, pathLength, visualize);
    }
    return;
}


int main(){
    test_PP();
    return 0;
}
