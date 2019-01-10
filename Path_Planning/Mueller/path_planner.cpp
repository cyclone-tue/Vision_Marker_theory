#include "path_planner.h"


Vars vars;
Params params;
Workspace work;
Settings settings;



namespace {
    
    // let N be the number of points on the path including start and end point.

    double timeInterval = 1;
    int number_of_points = 51;   // = N


    const double d_after = 0.5;
    const double v_after = 0.1;
    const double d_before = 0.5;
    const double v_in = 0.1;

}

int path_planner::getPathSize(){
    return number_of_points;
}

void path_planner::init(){
    return;
}

void path_planner::release(){
    return;
}

/*
Let the hoop coordinate frame be as follows:
The x-axis goes through the main symmetry axis of the hoop, positive direction given by the passage direction.
The y-axis is horizontal in the earth-frame, positive direction to the right, from the incoming drone point of view.
The z-axis is as in a righthanded coordinate frame.

The positive direction of the z-axis of the world frame is parallel to the gravitational acceleration.
All used frames stand still with respect to the earth.
*/
int path_planner::run(VectorXd currentState, VectorXd currentTorque, Vector3d hoopTransVec, Matrix3d hoopRotMat, MatrixXd& path, VectorXd& timeDiffs, MatrixXd& torques){
    // currentState = state of drone in world frame.
    // hoopTransVec = hoop position in world frame.
    // hoopRotMat = rotation from hoop frame to world frame.

    set_defaults();                 // in solver.c
    setup_indexing();               // in solver.c


    MatrixXd allConstraints = getConstraints(currentState, currentTorque, hoopTransVec, hoopRotMat);
    cout << allConstraints << endl;

    int waypoints = allConstraints.cols()/3 - 1;
    int totalLength = 0;

    MatrixXd pathPart(number_of_points, 12);
    VectorXd timeDiffsPart(number_of_points);
    MatrixXd torquesPart(number_of_points, 4);
    MatrixXd constraintsPart(6,3);		// constraints = [[x0, xd0, xdd0]; [y0, ... ]; [z0, ... ]; [x1, xd1, xdd1]; [y1, ... ]; [z1, ... ]]




    //path.resize(0,12);
    //timeDiffs.resize(0);
    //torques.resize(0,4);

    VectorXd beginState(12);
    beginState = currentState;
    VectorXd beginState_temp(12);
    for (int i = 0; i <= waypoints; i++) {							// iterate over path intervals.

        constraintsPart = allConstraints.block<6,3>(0, i*3); 			// get constraints for this path interval

        getPathSegment(beginState, constraintsPart, pathPart, timeDiffsPart, torquesPart);
        int points = pathPart.rows();
        totalLength += points;

        beginState_temp = pathPart.block<1,12>(pathPart.rows()-1, 0);
        beginState = beginState_temp.replicate(1,12);

        path = concatenate(path, pathPart);

        //MatrixXd path_temp(totalLength, 12);
        //path_temp.block(0,0, path.rows(), 12) = path.replicate(path.rows(), 12);
        //path_temp.block(totalLength-points-1, 0, points, 12) = pathPart.replicate(points, 12);
        //path.resize(totalLength, 12);
        //path = path_temp.replicate(totalLength, 12);


        timeDiffs = concatenateVec(timeDiffs, timeDiffsPart);

        //VectorXd timeDiffs_temp(totalLength);
        //timeDiffs_temp.block(0,0, timeDiffs.rows(), 1) = timeDiffs.replicate(timeDiffs.rows(),1);
        //timeDiffs_temp.block(totalLength-points-1, 0, points, 12) = timeDiffsPart.replicate(points,1);
        //cout << "a" << endl;
        //timeDiffs.resize(totalLength);
        //cout << "b" << endl;
        //timeDiffs = timeDiffs_temp.replicate(totalLength,1);


        torques = concatenate(torques, torquesPart);

        //MatrixXd torques_temp(totalLength,4);
        //torques_temp.block(0,0, torques.rows(), 1) = torques.replicate(torques.rows(), 4);
        //torques_temp.block(totalLength-points-1,0,points,4) = torquesPart.replicate(points, 4);
        //torques.resize(totalLength, 4);
        //torques = torques_temp.replicate(totalLength, 4);


    }

    return number_of_points*(waypoints + 1);
}

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
void path_planner::getPathSegment(VectorXd currentState, MatrixXd constraints, MatrixXd& path, VectorXd& timeDiffs, MatrixXd& torques){
    MatrixXd pos(51, 3);
    MatrixXd jerk(51, 3);

    Vector3d beginX = constraints.block<1,3>(0,0);
    Vector3d endX = constraints.block<1,3>(3,0);
    Vector3d beginY = constraints.block<1,3>(1,0);
    Vector3d endY = constraints.block<1,3>(4,0);
    Vector3d beginZ = constraints.block<1,3>(2,0);
    Vector3d endZ = constraints.block<1,3>(5,0);

    load_default_data(beginX, endX);            // x
    long num_iters_x = solve();                   // in solver.c
    for(int i = 0; i <= 50; i++){
        pos(i, 0) = *vars.z[i];
        jerk(i, 0) = *vars.jerk[i];
    }

    load_default_data(beginY, endY);            // y
    long num_iters_y = solve();                   // in solver.c
    for(int i = 0; i <= 50; i++){
        pos(i, 1) = *vars.z[i];
        jerk(i, 1) = *vars.jerk[i];
    }

    load_default_data(beginZ, endZ);            // z
    long num_iters_z = solve();                   // in solver.c
    for(int i = 0; i <= 50; i++){
        pos(i, 2) = *vars.z[i];
        jerk(i, 2) = *vars.jerk[i];
    }


    jerkToPath(currentState, pos, jerk, path, timeDiffs, torques);
}


MatrixXd path_planner::getConstraints(VectorXd currentState, VectorXd currentTorque, Vector3d hoopTransVec, Matrix3d hoopRotMat){

    // ders = [[x, xd, xdd]; [y, yd, ydd]; [z, zd, zdd]]

    Matrix3d currentDers = stateToPosDers(currentState, currentTorque);
    Matrix3d before_ders = get_before_ders(hoopTransVec, hoopRotMat);
    Matrix3d after_ders = get_after_ders(hoopTransVec, hoopRotMat);

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
Matrix3d path_planner::get_before_ders(Vector3d hoopTransVec, Matrix3d hoopRotMat){
    double d_before = -.5;
    double v_before = .5;

    Matrix3d R = hoopRotMat;

    // state relative to hoop before hoop
    Vector3d dist_in(d_before, 0, 0);				// hoop frame
    Vector3d vel_in(v_before, 0, 0);
    dist_in = hoopTransVec + R*dist_in;				// world frame
    vel_in = R*vel_in;

    Matrix3d ders;
    ders.block<3,1>(0,0) = dist_in;
    ders.block<3,1>(0,1) = vel_in;
    ders(0,2) = 0;
    ders(1,2) = 0;
    ders(2,2) = 0;

    return ders;
}

/*
return format is
 [[x, xd, xdd]; [y, yd, ydd]; [z, zd, zdd]]
*/
Matrix3d path_planner::get_after_ders(Vector3d hoopTransVec, Matrix3d hoopRotMat){
    double d_after = .5;
    double v_after = .5;

    Matrix3d R = hoopRotMat;

    // state relative to hoop after hoop
    Vector3d dist_out(d_after, 0, 0);				// hoop frame
    Vector3d vel_out(v_after, 0, 0);
    dist_out = hoopTransVec + R*dist_out;			// world frame
    vel_out = R*vel_out;

    Matrix3d ders;
    ders.block<3,1>(0,0) = dist_out;
    ders.block<3,1>(0,1) = vel_out;
    ders(0,2) = 0;
    ders(1,2) = 0;
    ders(2,2) = 0;

    return ders;
}



/*
return format is
 [[x, xd, xdd]; [y, yd, ydd]; [z, zd, zdd]]
*/
Matrix3d path_planner::stateToPosDers(VectorXd currentState, Vector4d currentTorque){

    Matrix3d ders;

    ders(0,0) = currentState[0];
    ders(1,0) = currentState[1];
    ders(2,0) = currentState[2];
    ders(0,1) = currentState[3];
    ders(1,1) = currentState[4];
    ders(2,1) = currentState[5];
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
void path_planner::jerkToPath(VectorXd beginState, MatrixXd pos, MatrixXd jerk, MatrixXd& path, VectorXd& timeDiffs, MatrixXd& torques){



    double dt = timeInterval/number_of_points;

    path.block<1,12>(0,0) = beginState;
    timeDiffs(0) = 0;


    for(int i = 1; i < number_of_points; i++){  // time = i*dt

        // calculate state at time
        path.block<1,3>(i,0) = pos.block<1,3>(i,0);
        path.block<1,3>(i,3) = path.block<1,3>(i-1,3) + dt*jerk.block<1,3>(i,0);    // very incorrect.

        double psi = 0;
        path(i,8) = psi;
        path(i,7) = atan(jerk(i,1)*sin(psi) + jerk(i,0)*cos(psi))/(jerk(i,2) - g);
        path(i,6) = atan((jerk(i,0)/jerk(i,1) - cos(psi)*tan(path(i,7)))*cos(path(i,7))/sin(psi));
        double phi = path(i,6);
        double theta = path(i,7);

        double phidot = (path(i,6) - path(i-1,6))/dt;
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
        torques(i,0) = (g - jerk(i,2))*m/(cos(phi)*cos(theta));
        torques(i,1) = pdot*Ix + (Iz - Iy)*q*r;
        torques(i,2) = pdot*Iy + (Ix - Iz)*p*r;
        torques(i,3) = pdot*Iz + (Iy - Ix)*p*q;


    }
}


void path_planner::load_default_data(Vector3d beginState, Vector3d endState) {
    double dt = timeInterval/number_of_points;
    params.A[0] = 1;
    params.A[1] = dt;
    params.A[2] = pow(dt,2)/2;
    params.A[3] = 0;
    params.A[4] = 1;
    params.A[5] = dt;
    params.A[6] = 0;
    params.A[7] = 0;
    params.A[8] = 1;
    params.B[0] = pow(dt,3)/6;
    params.B[1] = pow(dt,2)/2;
    params.B[2] = dt;

    params.selectVelocity[0] = 0;
    params.selectVelocity[1] = 1;
    params.selectVelocity[2] = 0;
    params.selectAcceleration[0] = 0;
    params.selectAcceleration[1] = 0;
    params.selectAcceleration[2] = 1;

    params.min_vel[0] = -1;                  // not in paper
    params.max_vel[0] = 1;                   // not in paper
    params.min_acc[0] = -1;
    params.max_acc[0] = 1;
    params.min_jerk[0] = -1;
    params.max_jerk[0] = 1;

    params.initial[0] = beginState[0];
    params.initial[1] = beginState[1];
    params.initial[2] = beginState[2];
    params.final[0] = endState[0];
    params.final[1] = endState[1];
    params.final[2] = endState[2];

    return;
}

MatrixXd arrayToEigen(double* array, int rows, int columns){
    MatrixXd eigenMat(rows, columns);

    for(int row = 0; row < rows; row++){
        for(int column = 0; column < columns; column++){
            eigenMat(row, column) = array[columns*row + column];
        }
    }

    return eigenMat;
}