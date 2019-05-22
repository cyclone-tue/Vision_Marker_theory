#include "path_planner.h"
#include "../../logging.h"
#include "spdlog/fmt/ostr.h"
#include "../../general.h"

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
    int n = 40;      // let n be the number as defined in CVXGEN.

    double d_before = -.5;
    double v_before = .5;
    double d_after = 1;
    double v_after = .5;

    double f_min = 0.1*g;
    double f_max = 2*g;

    double omega_max = 3;
}

typedef Matrix<double, 12, 1> Vector12d;
typedef Matrix<double, Dynamic, 12> MatrixX12;
typedef Matrix<double, Dynamic, 3> MatrixX3;


// ==== public functions ===


void path_planner::init(){

    pp_logger->info("Distance, velocity before and after the hoop are given by [{};{}] and [{};{}], respectively", d_before, v_before, d_after, v_after);
    pp_logger->info("Parameters f_min and f_max are given by {} and {}", f_min, f_max);
    pp_logger->info("Acceleration in x,y,z direction is limited by [{};{}], [{};{}] and [{};{}], respectively", getAccLimit('x')(0), getAccLimit('x')(1), getAccLimit('y')(0), getAccLimit('y')(1), getAccLimit('z')(0), getAccLimit('z')(1));
    //pp_logger->debug("The upper limit of the acceleration squared is {}", pow(getAccLimit('x')(1), 2) + pow(getAccLimit('y')(1), 2) + pow(getAccLimit('z')(0) - g, 2));
    //pp_logger->debug("f_max squared is {}", pow(f_max, 2));
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
Trajectory path_planner::run(Vector12d& currentState, Vector4d& currentTorque, Vector3d& hoopTransVec, Matrix3d& hoopRotMat){

    IOFormat rowFmt(StreamPrecision, 0, ", ", "; ", "", "", "[", "]");

    pp_logger->debug("currentState : {}", currentState.format(rowFmt));
    pp_logger->debug("currentTorque : {}", currentTorque.format(rowFmt));
    pp_logger->debug("hoopTransVec : {}", hoopTransVec.format(rowFmt));
    pp_logger->debug("hoopRotMat : \n{}", hoopRotMat);

    set_defaults();                 // in solver.c
    setup_indexing();               // in solver.c

    std::vector<Matrix3d> constraints = getConstraints(currentState, currentTorque, hoopTransVec, hoopRotMat); // constraints = [[x0, xd0, xdd0]; [y0, ... ]; [z0, ... ]; [x1, xd1, xdd1]; [y1, ... ]; [z1, ... ]]
    int waypoints = constraints.size() - 2;


    // just logging
    if(waypoints == 1){
        pp_logger->debug("constraints :\n {},\n {},\n {}", constraints.at(0), constraints.at(1), constraints.at(2));
    } else{
        pp_logger->debug("constraints :\n {},\n {}", constraints.at(0), constraints.at(1));
    }

    // set constraint states into the trajectory
    Trajectory traj;
    Vector3d zeroVec3 = {0,0,0};
    Vector4d zeroVec4 = {0,0,0,0};
    traj.appendState(currentState);
    traj.appendAcc(constraints.at(0).col(2));
    traj.appendJerk(zeroVec3);
    std::vector<int> marks;

    for(int i = 1; i < constraints.size(); i++){
        Vector12d conState;
        conState << constraints.at(i).col(0), constraints.at(i).col(1), 0,0,0, 0,0,0;
        traj.appendState(conState);
        traj.appendAcc(constraints.at(i).col(2));
        traj.appendJerk(zeroVec3);
        traj.appendTime(1);
        traj.appendTorque(zeroVec4);
    }
    traj.appendAll();
    traj.setBeginTorque(currentTorque);
    for(int t = 0; t < constraints.size(); t++) marks.push_back(traj.mark(t));

    //pp_logger->debug("With constraints:");
    //traj.log();
    //vpp_logger->flush();
    //pp_logger->flush();
    //v_logger->flush();


    for (int i = 0; i <= waypoints; i++) {							// iterate over path intervals.
        pp_logger->debug("Starting path segment {}", i+1);
        traj = gotoWaypoint(traj, marks.at(i), marks.at(i+1), (waypoints!=0 && i==0), hoopTransVec);
    }

    traj = fix_yaw(traj);

    pp_logger->debug("end of run()");
    pp_logger->flush();

    return traj;
}

Trajectory path_planner::fix_yaw(Trajectory traj){


    double beginYaw = traj.state(0)(8);
    int mark = traj.mark(0);

    double time = 0;
    while(true){
        traj.increment_mark(mark);
        double yaw = traj.state(mark)(8);
        double time = traj.time(mark);

        if(abs(beginYaw - yaw)/time < omega_max){
            int mark1 = traj.mark(0);
            for( ; traj.time(mark1) < time; traj.increment_mark(mark1)){
                traj.path(traj.index(mark1), 8) = beginYaw + (yaw - beginYaw)/time * traj.time(mark1);
            }
            break;
        }

    }

    return traj;
}


/*
void path_planner::removeRow(MatrixXd& matrix, int rowToRemove){

    unsigned int numRows = matrix.rows()-1;
    unsigned int numCols = matrix.cols();

    if( rowToRemove < numRows )
        matrix.block(rowToRemove,0,numRows-rowToRemove,numCols) = matrix.block(rowToRemove+1,0,numRows-rowToRemove,numCols);

    matrix.conservativeResize(numRows,numCols);
    return;
}

void path_planner::removeElement(VectorXd& vec, int indexToRemove){

    unsigned int length = vec.size()-1;

    if( indexToRemove < length )
        vec.block(indexToRemove,0,length-indexToRemove,1) = vec.block(indexToRemove+1,0,length-indexToRemove,1);

    vec.conservativeResize(length,1);
    return;
}*/

Trajectory path_planner::gotoWaypoint(Trajectory& traj, int mark0, int mark1, bool look, Vector3d look_at){

    // find a suitable upper time limit to start with.
    double t_max = 1;
    bool success = false;
    for(t_max = 1; t_max <= 2000; t_max *= 2){
        if(success = getPathSegment(t_max, traj, mark0, mark1, look, look_at)) break;
    }
    pp_logger->flush();
    if(not success) throw std::runtime_error("gotoWaypoint: No valid upper time limit exists.");     // throw some error;
    pp_logger->debug("=========================================================================== found upper time limit ============================================================");


    // optimize the time
    double t_min = 0;
    double epsilon = 0.01;
    for(double t = (t_max + t_min)/2; t_max - t_min >= epsilon; t = (t_max + t_min)/2){
        if(success = getPathSegment(t, traj, mark0, mark1, look, look_at)){
            t_max = t;
        } else{
            t_min = t;
        }
    }
    success = getPathSegment(t_max, traj, mark0, mark1, look, look_at);
    pp_logger->flush();
    if(not success) throw runtime_error("Time optimization failed.");
    pp_logger->debug("================================================================================ found optimal time =============================================================");

    //pp_logger->debug("segment with optimal time:");
    //traj.log();


    /*
    // improve time resolution, we keep using the original traj object.
    for(int segments = 2; traj.maxTimeDif() > 0.01; segments *= 2){

        std::vector<int> marks;     // create and save the marks that indicate the new segments(number of new segments is "segments").
        for(int seg = 0; seg <= segments; seg++){
            double time = seg* (traj.time(mark1) - traj.time(mark0))/segments;
            marks.push_back(traj.mark(time));
        }

        for(int markIndex = 0; markIndex < marks.size()-1; markIndex++){

            traj.log();

            pp_logger->debug("here");
            pp_logger->flush();

            int markA = marks.at(markIndex);
            int markB = marks.at(markIndex+1);

            pp_logger->debug("time difference is {}", (traj.time(markB) - traj.time(markA)));
            pp_logger->flush();
            if( not getPathSegment(1.5*(traj.time(markB) - traj.time(markA)), traj, markA, markB, look, look_at)){
                pp_logger->flush();
                throw runtime_error("Improving time resolution failed.");
            }
        }
        pp_logger->debug("================================================================================ number of segments between mark0 and mark1 should be 'segments'={}", segments);
        traj.log();
    }
    */

    return traj;
}



// ==== other functions ====

/*
MatrixXd path_planner::concatenate(std::vector<MatrixXd> parts) {
    int rows = 0;
    int cols = parts.at(0).cols();
    for (int i = 0; i < parts.size(); i++) {
        rows += parts.at(i).rows();
    }
    MatrixXd result(rows, parts.at(0).cols());
    int row = 0;
    for (int i = 0; i < parts.size(); i++) {
        result.block(row, 0, rows, cols) = parts.at(i);
        row += parts.at(i).rows();
    }
    return result;
}

VectorXd path_planner::concatenateVec(std::vector<VectorXd> parts) {
    int rows = 0;
    int cols = parts.at(0).cols();
    for (int i = 0; i < parts.size(); i++) {
        rows += parts.at(i).rows();
    }
    MatrixXd result(rows, parts.at(0).cols());
    int row = 0;
    for (int i = 0; i < parts.size(); i++) {
        result.block(row, 0, rows, cols) = parts.at(i);
        row += parts.at(i).rows();
    }
    return result;
}*/

/*
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
}*/

/*
currentState is size-12 vector.
constraints = [[x0, xd0, xdd0]; [y0, ... ]; [z0, ... ]; [x1, xd1, xdd1]; [y1, ... ]; [z1, ... ]]
 */

bool path_planner::getPathSegment(double time, Trajectory& traj, int mark0, int mark1, bool look, Vector3d look_at){

    bool success = true;

    Vector3d beginX; beginX << traj.state(mark0)(0), traj.state(mark0)(3), traj.acc(mark0)(0);
    Vector3d endX; endX << traj.state(mark1)(0), traj.state(mark1)(3), traj.acc(mark1)(0);
    Vector3d beginY; beginY << traj.state(mark0)(1), traj.state(mark0)(4), traj.acc(mark0)(1);
    Vector3d endY; endY << traj.state(mark1)(1), traj.state(mark1)(4), traj.acc(mark1)(1);
    Vector3d beginZ; beginZ << traj.state(mark0)(2), traj.state(mark0)(5), traj.acc(mark0)(2);
    Vector3d endZ; endZ << traj.state(mark1)(2), traj.state(mark1)(5), traj.acc(mark1)(2);
    //pp_logger->debug("x: {},{}, y:{},{}, z:{},{}", beginX, endX, beginY, endY, beginZ, endZ);
    MatrixX3 pos(n+2, 3);    // excluding end point
    MatrixX3 vel(n+2, 3);
    MatrixX3 acc(n+2, 3);
    MatrixX3 jerk(n+1, 3);

    VectorXd posCol(n+2);
    VectorXd velCol(n+2);
    VectorXd accCol(n+2);
    VectorXd jerkCol(n+1);

    success &= getSegment1D(time, beginX, endX, posCol, velCol, accCol, jerkCol, 'x');  // includes end point (should have n+2 elements)
    pos.col(0) = posCol;
    vel.col(0) = velCol;
    acc.col(0) = accCol;
    jerk.col(0) = jerkCol;
    success &= getSegment1D(time, beginY, endY, posCol, velCol, accCol, jerkCol, 'y');
    pos.col(1) = posCol;
    vel.col(1) = velCol;
    acc.col(1) = accCol;
    jerk.col(1) = jerkCol;
    success &= getSegment1D(time, beginZ, endZ, posCol, velCol, accCol, jerkCol, 'z');
    pos.col(2) = posCol;
    vel.col(2) = velCol;
    acc.col(2) = accCol;
    jerk.col(2) = jerkCol;

    pp_logger->debug("success: {} with time {}", success, time);
    traj.valid = success;
    if(not success){
        pp_logger->flush();
        return success;
    }

    jerkToPath(time, traj, pos, vel, acc, jerk, mark0, mark1, look, look_at);  // excludes end point

    return true;
}


bool path_planner::getSegment1D(double time, Vector3d& begin, Vector3d& end, VectorXd& pos, VectorXd& vel, VectorXd& acc, VectorXd& jerk, char dim){
    load_data(time, begin, end, dim);
    long num_iters = solve();   // in solver.c
    for(int i = 0; i <= n+1; i++) pos(i, 0) = vars.x[i][0];
    for(int i = 0; i <= n; i++) jerk(i, 0) = vars.jerk[i][0];
    for(int i = 0; i <= n+1; i++) vel(i, 0) = vars.x[i][1];
    for(int i = 0; i <= n+1; i++) acc(i, 0) = vars.x[i][2];
    return work.converged == 1;
}


std::vector<Matrix3d> path_planner::getConstraints(Vector12d& currentState, Vector4d& currentTorque, Vector3d& hoopTransVec, Matrix3d& hoopRotMat){
    // ders = [[x, xd, xdd]; [y, yd, ydd]; [z, zd, zdd]]
    Matrix3d currentDers = stateToPosDers(currentState, currentTorque);

    // ensure accelleration limits are satisfied
    Vector2d limX = getAccLimit('x');
    Vector2d limY = getAccLimit('y');
    Vector2d limZ = getAccLimit('z');
    double accX = currentDers(0, 2);
    double accY = currentDers(1, 2);
    double accZ = currentDers(2, 2);
    if(accX < limX(0) or accX > limX(1) or accY < limY(0) or accY > limY(1) or accZ < limZ(0) or accZ > limZ(1)){
        pp_logger->error("Current accelleration is out of bounds: {}, {}, {}", accX, accY, accZ);
        throw runtime_error("getConstraints(): Current accelleration is out of bounds.");
    }
    if(accX < limX(0)) currentDers(0, 2) = limX(0);
    if(accX > limX(1)) currentDers(0, 2) = limX(1);
    if(accY < limY(0)) currentDers(1, 2) = limY(0);
    if(accY > limY(1)) currentDers(1, 2) = limY(1);
    if(accZ < limZ(0)) currentDers(2, 2) = limZ(0);
    if(accZ > limZ(1)) currentDers(2, 2) = limZ(1);

    Matrix3d before_ders = get_ders_hoop_to_world(d_before, v_before, hoopTransVec, hoopRotMat);
    Matrix3d after_ders = get_ders_hoop_to_world(d_after, v_after, hoopTransVec, hoopRotMat);
    Matrix3d after_after_ders = get_ders_hoop_to_world(d_after + 1, 0, hoopTransVec, hoopRotMat);

    Vector3d pos_now = currentState.block<3,1>(0,0);
    Vector3d pos_in = before_ders.block<3,1>(0,0);
    Vector3d pos_out = after_ders.block<3,1>(0,0);
    double distance_to_second = (pos_out - pos_now).norm();
    double distance_between_first_second = (pos_out - pos_in).norm();

    std::vector<Matrix3d> allConstraints;
    if(distance_to_second < distance_between_first_second*1.2) {       //go to point after hoop
        allConstraints.push_back(currentDers);        // constraints of first trajectory part (currentState -> stateBeforeHoop)
        allConstraints.push_back(after_ders);
    }
    else{                                       // go to point before hoop.
        allConstraints.push_back(currentDers);		// constraints of first trajectory part (currentState -> stateBeforeHoop)
        allConstraints.push_back(before_ders);
        allConstraints.push_back(after_ders);// constraints of second trajectory part (stateBeforeHoop -> stateAfterHoop)
    }
    allConstraints.push_back(after_after_ders);
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
Matrix3d path_planner::stateToPosDers(const Vector12d& currentState, const Vector4d& currentTorque){
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

// i will include end point. dt[0] = 0.

void path_planner::jerkToPath(double time, Trajectory& fullTraj, MatrixX3& pos, MatrixX3& vel, MatrixX3& acc, MatrixX3& jerk, int mark0, int mark1, bool look, Vector3d look_at){

    double dt = time/(n+1);


    Vector12d state(12);
    Vector12d oldState = fullTraj.state(mark0);
    double timeDiff;
    Vector4d torque;

    Trajectory traj;
    traj.appendState(oldState);
    traj.appendAcc(fullTraj.acc(mark0));
    traj.appendJerk(fullTraj.jerk(mark0));

    for(int i = 1; i <= n+1; i++){  // time = i*dt
        int j = i-1;

        // calculate state at time
        state(0) = pos(i,0); state(1) = pos(i,1); state(2) = pos(i,2);
        state(3) = vel(i,0); state(4) = vel(i,1); state(5) = vel(i,2);

        double psi;
        if(look) psi = getYawToHoop(state.segment(0,3), look_at);
        else psi = fullTraj.state(mark0)(8);
        //double psi = 0;
        //double theta = atan(1/(g-acc(i,2)) * (acc(i,1)*sin(psi) - acc(i,0))/(1-2*pow(sin(psi),2)));
        //double phi = -atan(-acc(i,1)/(g-acc(i,2)) * cos(theta)/cos(psi) - tan(psi)*sin(theta));

        double theta = atan2(-(cos(psi)*acc(i,0) + sin(psi)*acc(i,1)), -(acc(i,2) - g));
        double phi = atan2(cos(theta)*(cos(psi)*acc(i,1) - sin(psi)*acc(i,0)), -(acc(i,2) - g));

        double phidot = (state(6) - oldState(6))/dt;       // this can be done better...
        double thetadot = (state(7) - oldState(7))/dt;
        double psidot = (state(8) - oldState(8))/dt;

        double p = phidot + (2*pow(sin(phi),2) - sin(theta))*psidot;
        double q = cos(psi)*thetadot + sin(phi)*cos(theta)*psidot;
        double r = cos(phi)*cos(theta)*psidot - sin(phi)*thetadot;

        state.tail(6) << phi,theta,psi, p,q,r;

        traj.appendState(state);
        traj.appendAcc(acc.row(i));
        traj.appendJerk(jerk.row(i));

        double pdot = (state(9) - oldState(9))/dt;
        double qdot = (state(10) - oldState(10))/dt;
        double rdot = (state(11) - oldState(11))/dt;

        // time interval
        traj.appendTime(dt);

        // calculate torques at time
        torque(0) = (g - jerk(j,2))*m/(cos(phi)*cos(theta));
        torque(1) = pdot*Ix + (Iz - Iy)*q*r;
        torque(2) = qdot*Iy + (Ix - Iz)*p*r;
        torque(3) = rdot*Iz + (Iy - Ix)*p*q;
        traj.appendTorque(torque);

        oldState = state;
    }
    traj.appendAll();
    fullTraj.replace(mark0, mark1, traj);

    return;
}


double path_planner::getYawToHoop(Vector3d pos, Vector3d pos_to_look_at){

    Vector3d difference = pos_to_look_at - pos;
    double lengthXY = sqrt(pow(difference(0), 2) + pow(difference(1), 2));
    double yaw = acos(difference(0) / lengthXY);
    if (difference(1) <= 0) yaw = -yaw;

    return yaw;
}

/*
bool path_planner::validTorques(MatrixXd& torques){
    double maxThrust = 25./4;  //Newton
    double minThrust = 0;
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
}*/


Vector2d path_planner::getAccLimit(char dim){
    //2*x^2 + (c*x + g)^2 = f^2;
    //2*x^2 + c^2*x^2 + 2*cxg + g^2 = f^2;
    //(x + cg/(2+c^2))^2 = c^2g^2/(2 + c^2)^2 + f^2 - g^2;
    double c = 1;
    double xddot_min = (c*g - sqrt( (2 + pow(c,2))*pow(f_max,2) - 2*pow(g,2) ))/(2+pow(c,2));
    double xddot_max = -1*xddot_min;
    double yddot_min = xddot_min;
    double yddot_max = xddot_max;

    double zddot_max = g-f_min;
    double zddot_min = c*xddot_min;

    Vector2d acc_limits;
    if(dim == 'x'){
        acc_limits << xddot_min, xddot_max;
    }
    if(dim == 'y'){
        acc_limits << yddot_min, yddot_max;
    }
    if(dim == 'z'){
        acc_limits << zddot_min, zddot_max;
    }
    return acc_limits;
}


void path_planner::load_data(double time, Vector3d beginState, Vector3d endState, char dim) {
    double dt = time/(n+1);

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


    params.select_acc[0] = 0.;
    params.select_acc[1] = 0.;
    params.select_acc[2] = 1.;




    Vector2d acc_limits = getAccLimit(dim);
    params.min_acc[0] = 0.8*acc_limits(0);
    params.acc_dif[0] = 0.8*acc_limits(1) - 0.8*acc_limits(0);


    double j_max = f_min*omega_max/sqrt(3);
    params.min_jerk[0] = -1*j_max;
    params.jerk_dif[0] = 2*j_max;

    params.initial[0] = beginState[0];
    params.initial[1] = beginState[1];
    params.initial[2] = beginState[2];

    params.final[0] = endState[0];
    params.final[1] = endState[1];
    params.final[2] = endState[2];

    //params.lambda[0] = 100000;
    //params.lambda[1] = 10000;
    //params.lambda[2] = 1000;
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
