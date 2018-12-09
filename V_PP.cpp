

#include "V_PP.h"


int runFrame(bool visualize, VectorXd currentState, VectorXd currentTorque, MatrixXd& path, VectorXd& timeDiffs, MatrixXd& torques){

    int pathLength = 0;

    Vector3d hoopTransVec;
    Matrix3d hoopRotMat;

    cout << "run vision" << endl;
    bool foundHoop = vision::run(hoopTransVec, hoopRotMat);


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


double* output_to_py(VectorXd currentState, VectorXd currentTorque, int* pathLength, bool visualize){
    double* db_p;  // stands for ...
    MatrixXd path;
    VectorXd timeDiffs;
    MatrixXd torques;

    *pathLength = runFrame(visualize, currentState, currentTorque, path, timeDiffs, torques);

    cout << *pathLength << endl;

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


int main(){

    setup();
    cout << "setup done" << endl;

    VectorXd currentState(12);
    VectorXd currentTorque(4);
    int* pathLength = new int(1);
    bool visualize = false;
    currentState << 0,0,0, 0,0,0, 0,0,0, 0,0,0;
    currentTorque << 0,0,0,0;

    double *db_p;
    //for(int i = 0; i<=1000; i++) {
    while(true){
        db_p = output_to_py(currentState, currentTorque, pathLength, visualize);
    }

    return 0;
}


