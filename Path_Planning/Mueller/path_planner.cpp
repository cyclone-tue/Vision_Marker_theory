#include "path_planner.h"


#include "../../constants.h"

extern "C"
{
#include "cvxgen/solver.h"
}

Vars vars;
Params params;
Workspace work;
Settings settings;
double m;

// let N be the number of points on the path including start and end point.

double timeInterval;
int number_of_points;

int path_planner::getPathSize(){
    return number_of_points;
}

void path_planner::init(){
    return;
}

void path_planner::release(){
    return;
}

int path_planner::run(VectorXd currentState, VectorXd currentTorque, Vector3d hoopTransVec, Matrix3d hoopRotMat, MatrixXd path, MatrixXd timeDiffs, MatrixXd torques){
    set_defaults();                 // in solver.c
    setup_indexing();               // in solver.c

    cout << "currentState:" << endl;
    cout << currentState << endl;
    cout << "currentTorque:" << endl;
    cout << currentTorque << endl;
    cout << "hoopTransVec:" << endl;
    cout << hoopTransVec << endl;
    cout << "hoopRotMat:" << endl;
    cout << hoopRotMat << endl;


    // beginX, endX are built as [x, xd, xdd]
    Vector3d beginX;
    Vector3d beginY;
    Vector3d beginZ;
    Vector3d endX;
    Vector3d endY;
    Vector3d endZ;
    stateToPosDers(currentState, currentTorque, beginX, beginY, beginZ);
    getEndStatePosDers(currentState, hoopTransVec, hoopRotMat, endX, endY, endZ);



    MatrixXd pos(51, 3);
    MatrixXd jerk(51, 3);

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

    jerkToPath(currentState, pos, jerk, timeInterval, number_of_points, path, timeDiffs, torques);

    return true;
}

void path_planner::stateToPosDers(VectorXd currentState, Vector4d currentTorque, Vector3d beginX, Vector3d beginY, Vector3d beginZ){
    beginX[0] = currentState[0];
    beginY[0] = currentState[1];
    beginZ[0] = currentState[2];
    beginX[1] = currentState[3];
    beginY[1] = currentState[4];
    beginZ[1] = currentState[5];
    double phi = currentState[6];
    double theta = currentState[7];
    double psi = currentState[8];
    double T = currentTorque[0];    //thrust
    beginX[2] = -(sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta))*T/m;
    beginY[2] = -(cos(phi)*sin(psi)*sin(theta) - cos(psi)*sin(psi))*T/m;
    beginZ[2] = -cos(phi)*cos(theta)*T/m + g;
}


void path_planner::getEndStatePosDers(VectorXd currentState, Vector3d hoopTransVec, Matrix3d hoopRotMat, Vector3d endX, Vector3d endY, Vector3d endZ) {
    double d_before = 1;
    double v_before = 1;
    double d_after = 1;
    double v_after = 1;

    Matrix3d R = hoopRotMat;

    // state relative to hoop before hoop
    Vector3d dist_in(d_before, 0, 0);				// hoop frame
    Vector3d vel_in(v_before, 0, 0);
    dist_in = hoopTransVec + R*dist_in;				// world frame
    vel_in = R*vel_in;

    // state relative to hoop after hoop
    Vector3d dist_out(d_after, 0, 0);				// hoop frame
    Vector3d vel_out(v_after, 0, 0);
    dist_out = hoopTransVec + R*dist_out;			// world frame
    vel_out = R*vel_out;

    double distance_to_first = (dist_in - currentState).norm();
    double distance_to_second = (dist_out, currentState).norm();
    double distance_between_first_second = (dist_out, dist_in).norm();


    if(distance_to_second < distance_between_first_second*1.2) {
        endX[0] = dist_out[0];
        endY[0] = dist_out[1];
        endZ[0] = dist_out[2];
        endX[1] = vel_out[0];
        endY[1] = vel_out[1];
        endZ[1] = vel_out[2];
        endX[2] = 0;
        endY[2] = 0;
        endZ[2] = 0;
    } else {
        endX[0] = dist_in[0];
        endY[0] = dist_in[1];
        endZ[0] = dist_in[2];
        endX[1] = vel_in[0];
        endY[1] = vel_in[1];
        endZ[1] = vel_in[2];
        endX[2] = 0;
        endY[2] = 0;
        endZ[2] = 0;
    }

    return;
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
void path_planner::jerkToPath(VectorXd beginState, MatrixXd pos, MatrixXd jerk, double timeInterval, double number_of_points, MatrixXd path, VectorXd timeDiffs, MatrixXd torques){
    double dt = timeInterval/number_of_points;

    path.block<1,12>(0,0) = beginState;

    for(int i = 1; i < number_of_points; i++){  // time = i*dt

        // calculate state at time
        path.block<1,3>(i,0) = pos.block<1,3>(i,0);
        path.block<1,3>(i,3) = path.block<1,3>(i-1,3) + dt*jerk.block<1,3>(i,0);

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