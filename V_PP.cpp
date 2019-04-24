

#include "V_PP.h"
#include "spdlog/fmt/ostr.h"



// ==== functions to be called from python scripts ====


namespace spd = spdlog;

std::shared_ptr<spdlog::logger> vpp_logger;
std::shared_ptr<spdlog::logger> pp_logger;
std::shared_ptr<spdlog::logger> v_logger;
Gnuplot gp;

void setup(const char* camera_calibration_file){

    // log levels: trace, debug, info, warn, error, critical, off.

    // create the sinks
    auto console_sink = std::make_shared<spd::sinks::stdout_color_sink_mt>();
    auto vpp_sink = std::make_shared<spd::sinks::basic_file_sink_mt>("logs/V_PP.txt");
    console_sink->set_level(spdlog::level::info);
    vpp_sink->set_level(spdlog::level::trace);

    std::vector<spdlog::sink_ptr> sinks;
    sinks.push_back(console_sink);
    sinks.push_back(vpp_sink);


    // create the loggers
    vpp_logger = std::make_shared<spdlog::logger>("V_PP", begin(sinks), end(sinks));
    pp_logger = std::make_shared<spdlog::logger>("PP", begin(sinks), end(sinks));
    v_logger = std::make_shared<spdlog::logger>("V", begin(sinks), end(sinks));

    vpp_logger->set_level(spdlog::level::trace);
    pp_logger->set_level(spdlog::level::trace);
    v_logger->set_level(spdlog::level::trace);


    vpp_logger->info("=============================== Running setup() ===");

    vision::setupVariables(0, camera_calibration_file);
    path_planner::init();


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
bool output_to_py(double* output, double* currentStateArray, double* currentTorqueArray, int* pathLength, bool visualize){

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
        MatrixXd outputInfo(*pathLength, 12+1+4);
        outputInfo << path, timeDiffs, torques;

        vpp_logger->debug("path: \n{}", outputInfo);


        //copy path to output array
        double output_array[*pathLength][12 + 1 + 4];
        Map<MatrixXd>(output, outputInfo.rows(), outputInfo.cols()) = outputInfo;
        //output_ptr = &output_array[0][0];
        //return output_ptr;
        return true;
    }
    
    return false;
}


bool runFrame(VectorXd& currentState, Vector4d& currentTorque, MatrixXd& path, VectorXd& timeDiffs, MatrixXd& torques){

    bool success = false;
    Vector3d hoopTransVec;
    Matrix3d hoopRotMat;

    bool foundHoop = vision::run(currentState, hoopTransVec, hoopRotMat);       // should be returned in world frame, instead of body frame.
    if(foundHoop){
        success = path_planner::run(currentState, currentTorque, hoopTransVec, hoopRotMat, path, timeDiffs, torques);
        if(not success){
            vpp_logger->error("Path planning was unsuccessful...");
        }
    }
    return success;
}

double *output_simple_path(double* currentStateArray, double* currentTorqueArray, int* pathLength, bool visualize) {
    VectorXd currentState(12);
    Vector3d hoopTransVec;
    Matrix3d hoopRotMat;
    currentState = arrayToEigen(currentStateArray, 12);
    bool foundHoop = vision::run(currentState, hoopTransVec, hoopRotMat);
    if(foundHoop){
        Matrix3d beforeData = path_planner::get_ders_hoop_to_world(-0.5, 1, hoopTransVec, hoopRotMat);
        Matrix3d afterData = path_planner::get_ders_hoop_to_world(0.5, 1, hoopTransVec, hoopRotMat);
        cout << "Before point: " << beforeData << endl;
        cout << "After point: " << afterData << endl;
        *pathLength = 3;
        MatrixXd outputPath(*pathLength,3);
        outputPath.row(0) = beforeData.col(0);
        outputPath.row(1) = hoopTransVec;
        outputPath.row(2) = afterData.col(0);
        cout << "Test path: " << outputPath << endl;
        double output_array[*pathLength][12 + 1 + 4];
        Map<MatrixXd>(&output_array[0][0], outputPath.rows(), outputPath.cols()) = outputPath;
        double* output_ptr;
        output_ptr = &output_array[0][0];
        return output_ptr;
    }

    *pathLength = 0;
    return nullptr;
}

VectorXd arrayToEigen(double* array, int length){
    VectorXd vector(length);
    for(int i = 0; i < length; i++){
        vector(i) = array[i];
    }
    return vector;
}


void test_PP(){

    setup("Vision/laptop_calibration.txt");
    vpp_logger->info("Starting test_PP()");


    VectorXd currentState(12);  // input variables
    Vector4d currentTorque;
    Vector3d hoopTransVec;
    Matrix3d hoopRotMat;
    MatrixXd path(0,12);        // output variables
    VectorXd timeDiffs(0);
    MatrixXd torques(0,4);

    currentState << 5,4,1, 0,0,0, 0,0,0, 0,0,0;         // input to the path planner
    currentTorque << 20,0,0,0;
    hoopTransVec << 10, -3, 2;
    hoopRotMat << 1,0,0, 0,1,0, 0,0,1;

    bool success = path_planner::run(currentState, currentTorque, hoopTransVec, hoopRotMat, path, timeDiffs, torques);

    MatrixXd thrusts(torques.rows(), 4);
    for(int i = 0; i < torques.rows(); i++){
        Vector4d torquesRow = torques.block<1,4>(i,0);
        thrusts.block<1,4>(i,0) = torquesToThrusts(torquesRow);
    }

    MatrixXd outputInfo(path.rows(), 12+1+4+4);
    outputInfo << path, timeDiffs, torques, thrusts;

    vpp_logger->debug("path, timeDiffs, torques: \n{}", outputInfo);


    showPathInteractive(path, timeDiffs, hoopTransVec, hoopRotMat);


    if(not success){
        vpp_logger->error("Test was unsuccessful...");
    }
    vpp_logger->info("End of test.");
    return;
}

void test_V_PP(){
    setup("Vision/laptop_calibration.txt");
    vpp_logger->info("Starting test_V_PP()");

    double currentState [12] = {0,0,0, 0,0,0, 0,0,0, 0,0,0};
    double currentTorque [4] = {20,0,0,0};
    int* pathLength = new int(1);
    bool visualize = true;

    double *output_ptr;   // contains path, timeDiffs, torques
    while(true){
        output_ptr = output_simple_path(currentState, currentTorque, pathLength, visualize);
    }
    return;
}


int main(){
    test_V_PP();
    return 0;
}

