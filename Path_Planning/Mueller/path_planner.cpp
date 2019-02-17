#include "path_planner.h"


Vars vars;
Params params;
Workspace work;
Settings settings;



/*
Let the hoop coordinate frame be as follows:
The x-axis goes through the main symmetry axis of the hoop, positive direction given by the passage direction.
The y-axis is horizontal in the earth-frame, positive direction to the right, from the incoming drone point of view.
The z-axis is as in a righthanded coordinate frame.

The positive direction of the z-axis of the world frame is parallel to the gravitational acceleration.
All used frames stand still with respect to the earth.
*/


namespace {
    double timeInterval = 10;
    int n = 35;      // let n be the number as defined in CVXGEN.

    double d_before = -.5;
    double v_before = .5;
    double d_after = .5;
    double v_after = .5;
}


// ==== public functions ===


void path_planner::init(){
    return;
}

void path_planner::release(){
    return;
}

/*
currentState = state of drone in world frame.
hoopTransVec = hoop position in world frame.
hoopRotMat = rotation from hoop frame to world frame.
*/
bool path_planner::run(VectorXd& currentState, Vector4d& currentTorque, Vector3d& hoopTransVec, Matrix3d& hoopRotMat, MatrixXd& path, VectorXd& timeDiffs, MatrixXd& torques){

    MatrixXd pathPart(n + 1, 12);       // including begin and excluding end point.
    VectorXd timeDiffsPart(n + 1);
    MatrixXd torquesPart(n + 1, 4);
    MatrixXd constraintsPart(6, 3);     // constraints = [[x0, xd0, xdd0]; [y0, ... ]; [z0, ... ]; [x1, xd1, xdd1]; [y1, ... ]; [z1, ... ]]
    VectorXd beginState(12);
    VectorXd beginState_temp(12);
    bool success = true;

    set_defaults();                 // in solver.c
    setup_indexing();               // in solver.c

    // set up the constraints
    MatrixXd allConstraints = getConstraints(currentState, currentTorque, hoopTransVec, hoopRotMat);
    writeDebug("constraints:\n");
    writeDebug(allConstraints);
    int waypoints = (int) allConstraints.cols() / 3 - 1;

    beginState = currentState;
    for (int i = 0; i <= waypoints; i++) {							// iterate over path intervals.
        constraintsPart = allConstraints.block<6,3>(0, i*3);

        double t_max = 30;
        double t_min = 1;
        double epsilon = 0.1;
        for(double t = (t_max + t_min)/2; t_max - t_min <= epsilon; t = (t_max + t_min)/2){
            if(getPathSegment(t_max, beginState, constraintsPart, pathPart, timeDiffsPart, torquesPart)){
                t_max = t;
            } else{
                t_min = t;
            }
        }
        success &= getPathSegment(t_max, beginState, constraintsPart, pathPart, timeDiffsPart, torquesPart);


        // set up beginState for next iteration
        beginState_temp = pathPart.block<1,12>(pathPart.rows()-1, 0);   // never calculated correctly due to incomplete jerkToPath().
        beginState = beginState_temp.replicate(1,12);
        beginState.block<3,1>(0,0) = constraintsPart.block<3,1>(3,0);
        beginState.block<3,1>(0,3) = constraintsPart.block<3,1>(3,1);

        path = concatenate(path, pathPart);
        timeDiffs = concatenateVec(timeDiffs, timeDiffsPart);
        torques = concatenate(torques, torquesPart);
    }
    return success;
}


// ==== other functions ====


MatrixXd path_planner::concatenate(MatrixXd& one, MatrixXd& two){
    MatrixXd result(one.rows() + two.rows(), one.cols());
    result.block(0,0, one.rows(), one.cols()) = one.replicate(one.rows(), one.cols());
    result.block(one.rows(), 0, two.rows(), two.cols()) = two.replicate(two.rows(), two.cols());
    return result;
}

VectorXd path_planner::concatenateVec(VectorXd& one, VectorXd& two){
    VectorXd result(one.size() + two.size());
    result.block(0,0, one.size(), 1) = one.replicate(one.size(),1);
    result.block(one.size(), 0, two.size(), 1) = two.replicate(two.size(), 1);
    return result;
}

/*
currentState is size-12 vector.
constraints = [[x0, xd0, xdd0]; [y0, ... ]; [z0, ... ]; [x1, xd1, xdd1]; [y1, ... ]; [z1, ... ]]
 */
bool path_planner::getPathSegment(double time, VectorXd& currentState, MatrixXd& constraints, MatrixXd& path, VectorXd& timeDiffs, MatrixXd& torques){

    Vector3d beginX = constraints.block<1,3>(0,0);
    Vector3d endX = constraints.block<1,3>(3,0);
    Vector3d beginY = constraints.block<1,3>(1,0);
    Vector3d endY = constraints.block<1,3>(4,0);
    Vector3d beginZ = constraints.block<1,3>(2,0);
    Vector3d endZ = constraints.block<1,3>(5,0);
    MatrixXd pos(n+2, 3);    // excluding end point
    MatrixXd vel(n+2, 3);
    MatrixXd acc(n+2, 3);
    MatrixXd jerk(n+1, 3);

    VectorXd posCol(n+2);
    VectorXd velCol(n+2);
    VectorXd accCol(n+2);
    VectorXd jerkCol(n+1);
    bool success = true;

    success &= getSegment1D(time, beginX, endX, posCol, velCol, accCol, jerkCol);  // excludes end point
    pos.col(0) = posCol;
    vel.col(0) = velCol;
    acc.col(0) = accCol;
    jerk.col(0) = jerkCol;
    success &= getSegment1D(time, beginY, endY, posCol, velCol, accCol, jerkCol);
    pos.col(1) = posCol;
    vel.col(1) = velCol;
    acc.col(1) = accCol;
    jerk.col(1) = jerkCol;
    success &= getSegment1D(time, beginZ, endZ, posCol, velCol, accCol, jerkCol);
    pos.col(2) = posCol;
    vel.col(2) = velCol;
    acc.col(2) = accCol;
    jerk.col(2) = jerkCol;

    //writeDebug("acc: \n");
    //writeDebug(acc);
    writeDebug("jerk: \n");
    writeDebug(jerk);

    success &= jerkToPath(time, currentState, pos, vel, acc, jerk, path, timeDiffs, torques);  // excludes end point


    return success;
}


bool path_planner::getSegment1D(double time, Vector3d& begin, Vector3d& end, VectorXd& pos, VectorXd& vel, VectorXd& acc, VectorXd& jerk){
    load_data(time, begin, end);
    long num_iters = solve();   // in solver.c
    for(int i = 0; i <= n+1; i++) pos(i, 0) = vars.x[i][0];
    for(int i = 0; i <= n; i++) jerk(i, 0) = vars.jerk[i][0];
    for(int i = 0; i <= n+1; i++) vel(i, 0) = vars.x[i][1];
    for(int i = 0; i <= n+1; i++) acc(i, 0) = vars.x[i][2];
    return work.converged == 1;
}


MatrixXd path_planner::getConstraints(VectorXd& currentState, Vector4d& currentTorque, Vector3d& hoopTransVec, Matrix3d& hoopRotMat){
    // ders = [[x, xd, xdd]; [y, yd, ydd]; [z, zd, zdd]]
    Matrix3d currentDers = stateToPosDers(currentState, currentTorque);
    Matrix3d before_ders = get_ders_hoop_to_world(d_before, v_before, hoopTransVec, hoopRotMat);
    Matrix3d after_ders = get_ders_hoop_to_world(d_after, v_after, hoopTransVec, hoopRotMat);

    Vector3d pos_now = currentState.block<3,1>(0,0);
    Vector3d pos_in = before_ders.block<3,1>(0,0);
    Vector3d pos_out = after_ders.block<3,1>(0,0);
    double distance_to_second = (pos_out - pos_now).norm();
    double distance_between_first_second = (pos_out - pos_in).norm();

    MatrixXd allConstraints(6,6);
    if(distance_to_second < distance_between_first_second*1.2) {       //go to point after hoop
        allConstraints.resize(6,3);
        allConstraints.block<3, 3>(0, 0) = currentDers;        // constraints of first trajectory part (currentState -> stateBeforeHoop)
        allConstraints.block<3, 3>(3, 0) = after_ders;
    }
    else{                                       // go to point before hoop.
        allConstraints.block<3, 3>(0, 0) = currentDers;		// constraints of first trajectory part (currentState -> stateBeforeHoop)
        allConstraints.block<3, 3>(3, 0) = before_ders;
        allConstraints.block<3, 3>(0, 3) = before_ders;		// constraints of second trajectory part (stateBeforeHoop -> stateAfterHoop)
        allConstraints.block<3, 3>(3, 3) = after_ders;
    }
    return allConstraints;
}

/*
return format is
 [[x, xd, xdd]; [y, yd, ydd]; [z, zd, zdd]]
*/
Matrix3d path_planner::get_ders_hoop_to_world(double dist, double vel, Vector3d& hoopTransVec, Matrix3d& hoopRotMat){
    Matrix3d R = hoopRotMat;

    // state before hoop
    Vector3d dist_vec(dist, 0, 0);				// hoop frame
    Vector3d vel_vec(vel, 0, 0);
    dist_vec = hoopTransVec + R*dist_vec;		// world frame
    vel_vec = R*vel_vec;

    Matrix3d ders;
    ders.block<3,1>(0,0) = dist_vec;
    ders.block<3,1>(0,1) = vel_vec;
    ders.block<3,1>(0,2) << 0,0,0;
    return ders;
}


/*
return format is
 [[x, xd, xdd]; [y, yd, ydd]; [z, zd, zdd]]
*/
Matrix3d path_planner::stateToPosDers(VectorXd& currentState, Vector4d& currentTorque){
    Matrix3d ders;
    ders.block<3,1>(0,0) = currentState.block<3,1>(0,0);
    ders.block<3,1>(0,1) = currentState.block<3,1>(3,0);
    double phi = currentState[6];
    double theta = currentState[7];
    double psi = currentState[8];
    double T = currentTorque[0];    //thrust
    ders(0,2) = -(sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta))*T/m;
    ders(1,2) = -(cos(phi)*sin(psi)*sin(theta) - cos(psi)*sin(psi))*T/m;
    ders(2,2) = -cos(phi)*cos(theta)*T/m + g;
    return ders;
}


/*

The input consists of:
beginState is a vector of length 12.
pos is a matrix of size (N,3) , each row consisting of [x,y,z] positions.
jerk is a matrix of size (N,3), each row consisting of [x,y,z] jerks.

The output consists of:
path is a N by 12 matrix, each row being a state.
timeDiffs is a vector of length N-1, with the time intervalls between the states in path.
torques is a N by 4 matrix each row being [thrust, torquex, torquey, torquez].

*/
bool path_planner::jerkToPath(double time, VectorXd& beginState, MatrixXd& pos, MatrixXd& vel, MatrixXd& acc, MatrixXd& jerk, MatrixXd& path, VectorXd& timeDiffs, MatrixXd& torques){

    double dt = time/(n+1);

    path.block<1,12>(0,0) = beginState;
    timeDiffs(0) = 0;


    for(int i = 1; i <= n; i++){  // time = i*dt
        int j = i-1;
        // calculate state at time
        path.block<1,3>(i,0) = pos.block<1,3>(i,0);
        path.block<1,3>(i,3) = vel.block<1,3>(i,3);

        double psi = 0;
        path(i,8) = psi;
        path(i,7) = atan(1/(g-acc(i,2)) * (acc(i,1)*sin(psi) - acc(i,0))/(1-2*pow(sin(psi),2)));
        double theta = path(i,7);
        path(i,6) = atan(-acc(i,1)/(g-acc(i,2)) * cos(theta)/cos(psi) - tan(psi)*sin(theta));
        double phi = path(i,6);

        double phidot = (path(i,6) - path(i-1,6))/dt;       // this can be done better...
        double thetadot = (path(i,7) - path(i-1,7))/dt;
        double psidot = (path(i,8) - path(i-1,8))/dt;

        path(i,9) = phidot + (2*pow(sin(phi),2) - sin(theta))*psidot;
        path(i,10) = cos(psi)*thetadot + sin(phi)*cos(theta)*psidot;
        path(i,11) = cos(phi)*cos(theta)*psidot - sin(phi)*thetadot;
        double p = path(i,9);
        double q = path(i,10);
        double r = path(1,11);

        double pdot = (path(i,9) - path(i-1,9))/dt;
        double qdot = (path(i,10) - path(i-1,10))/dt;
        double rdot = (path(i,11) - path(i-1,11))/dt;

        // time interval
        timeDiffs(i) = dt;

        // calculate torques at time
        torques(i,0) = (g - jerk(j,2))*m/(cos(phi)*cos(theta));
        torques(i,1) = pdot*Ix + (Iz - Iy)*q*r;
        torques(i,2) = pdot*Iy + (Ix - Iz)*p*r;
        torques(i,3) = pdot*Iz + (Iy - Ix)*p*q;

    }

    return validTorques(torques);

}


bool path_planner::validTorques(MatrixXd& torques){
    double maxThrust = 100;
    double minThrust = 1;
    for(int i = 0; i < torques.rows(); i++){
        Vector4d torquesRow = torques.row(i);
        Vector4d thrusts = torquesToThrusts(torquesRow);
        for(int j = 0; j <= 3; j++){
            if(thrusts(j) < minThrust || thrusts(j) > maxThrust){
                return false;
            }
        }
    }
    return true;
}

Vector4d path_planner::torquesToThrusts(Vector4d& torques){
    
}


void path_planner::load_data(double time, Vector3d beginState, Vector3d endState) {
    double dt = time/(n+1);
    /*
    params.A[0] = 1.;
    params.A[1] = dt;
    params.A[2] = pow(dt,2)/2.;
    params.A[3] = 0.;
    params.A[4] = 1.;
    params.A[5] = dt;
    params.A[6] = 0.;
    params.A[7] = 0.;
    params.A[8] = 1.;
    params.B[0] = pow(dt,3)/6.;
    params.B[1] = pow(dt,2)/2.;
    params.B[2] = dt;
     */

    params.A[0] = 1;
    params.A[1] = 0;
    params.A[2] = 0;
    params.A[3] = dt;
    params.A[4] = 1;
    params.A[5] = 0;
    params.A[6] = pow(dt,2)/2;
    params.A[7] = dt;
    params.A[8] = 1;
    params.B[0] = pow(dt,3)/6.;
    params.B[1] = pow(dt,2)/2.;
    params.B[2] = dt;


    //params.select_acc[0] = 0.;
    //params.select_acc[1] = 0.;
    //params.select_acc[2] = 1.;

    //params.min_acc[0] = -1.;
    //params.acc_dif[0] = 2.;
    //params.min_jerk[0] = -1.;
    //params.jerk_dif[0] = 2.;

    params.initial[0] = beginState[0];
    params.initial[1] = beginState[1];
    params.initial[2] = beginState[2];

    params.final[0] = endState[0];
    params.final[1] = endState[1];
    params.final[2] = endState[2];

    params.lambda[0] = 100000;
    params.lambda[1] = 10000;
    params.lambda[2] = 1000;
    //params.mu[0] = 0;
    //params.mu[1] = 0;
    //params.mu[2] = 100000;
    return;
}

/*
MatrixXd arrayToEigen(double* array, int rows, int columns){
    MatrixXd eigenMat(rows, columns);

    for(int row = 0; row < rows; row++){
        for(int column = 0; column < columns; column++){
            eigenMat(row, column) = array[columns*row + column];
        }
    }

    return eigenMat;
}
*/