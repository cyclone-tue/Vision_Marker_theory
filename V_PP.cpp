

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
    //path_planner::release();
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

    Vector12d currentState;      // input variables
    Vector4d currentTorque;
    double* output_ptr;             // output variables
    MatrixXd path(0,12);
    VectorXd timeDiffs(0);
    MatrixXd torques(0,4);

    currentState = arrayToEigen(currentStateArray, 12);
    currentTorque = arrayToEigen(currentTorqueArray, 4);

    Trajectory traj = runFrame(currentState, currentTorque, visualize);

    if(traj.valid) {
        *pathLength = path.rows();
        MatrixXd outputInfo(*pathLength, 3);
        outputInfo = path.block(0,0, path.rows(),3);

        vpp_logger->debug("path: \n{}", outputInfo);


        //copy path to output array
        double output_array[*pathLength][12 + 1 + 4];
        Map<MatrixXd>(&output_array[0][0], outputInfo.rows(), outputInfo.cols()) = outputInfo;
        output_ptr = &output_array[0][0];
        return output_ptr;
    }
    
    return nullptr;
}


Trajectory runFrame(Vector12d& currentState, Vector4d& currentTorque, bool visualize){

    bool success = false;
    Vector3d hoopTransVec;
    Matrix3d hoopRotMat;
    Trajectory traj;

    bool foundHoop = vision::run(currentState, hoopTransVec, hoopRotMat);       // should be returned in world frame, instead of body frame.
    if(foundHoop){
        traj = path_planner::run(currentState, currentTorque, hoopTransVec, hoopRotMat);
        if(not traj.valid){
            vpp_logger->error("Path planning was unsuccessful...");
        }
    }

    if(visualize){
        runVisualize(currentState, traj, hoopTransVec, hoopRotMat, traj.valid);
    }

    return traj;
}

VectorXd arrayToEigen(double* array, int length){
    VectorXd vector(length);
    for(int i = 0; i < length; i++){
        vector(i) = array[i];
    }
    return vector;
}


void test_PP(){

    //setup("Vision/laptop_calibration.txt");
    vpp_logger->info("Starting test_PP()");


    Vector12d currentState;  // input variables
    Vector4d currentTorque;
    Vector3d hoopTransVec;
    Matrix3d hoopRotMat;


    currentState << 1,2,1, 0,3,0, 0,0,0, 0,0,0;         // input to the path planner
    currentTorque << 20,0,0,0;
    hoopTransVec << 10, -3, 2;
    hoopRotMat << .5*sqrt(2),.5*sqrt(2),0, .5*sqrt(2),-.5*sqrt(2),0, 0,0,1;

    Trajectory traj = path_planner::run(currentState, currentTorque, hoopTransVec, hoopRotMat);
    if(not traj.valid || traj.time(-1) == 0){
        vpp_logger->error("Test was unsuccessful...");
        vpp_logger->info("End of test.");
        vpp_logger->flush();
        return;
    }
    traj.log();


    MatrixXd thrusts(traj.torques.rows(), 4);
    for(int i = 0; i < traj.torques.rows(); i++){
        Vector4d torquesRow = traj.torques.block<1,4>(i,0);
        thrusts.block<1,4>(i,0) = torquesToThrusts(torquesRow);
    }

    MatrixXd outputInfo(traj.path.rows(), 12+1+4+4);
    outputInfo << traj.path, traj.times, traj.torques, thrusts;

    vpp_logger->debug("path, timeDiffs, torques: \n{}", outputInfo);


    showPathInteractive(traj, hoopTransVec, hoopRotMat);

    vpp_logger->info("End of test.");
    return;
}

/*
void test_V_PP(){
    setup("Vision/laptop_calibration.txt");
    vpp_logger->info("Starting test_V_PP()");

    double currentState [12] = {0,0,0, 0,0,0, 0,0,0, 0,0,0};
    double currentTorque [4] = {20,0,0,0};
    int* pathLength = new int(1);
    bool visualize = true;

    double *output_ptr;   // contains path, timeDiffs, torques
    while(true){
        output_ptr = output_to_py(currentState, currentTorque, pathLength, visualize);
    }
    return;
}
*/

int main(){
    setup("Vision/laptop_calibration.txt");
    test_PP();
    return 0;
}

