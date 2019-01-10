#include "path_planner.h"


namespace {
    const double d_after = 1;
    const double v_after = 1;
    const double d_before = -2;
    const double v_in = 1;
}





/*
*/
void path_planner::getPathSegment(MatrixXd& constraints, double startYaw, Vector3d& pos_to_look_at, bool look_at_pos, MatrixXd& path, VectorXd& timeDiffs, MatrixXd& torques){
    // constraints = [[x0, xd0, xdd0]; [y0, ... ]; [z0, ... ]; [x1, xd1, xdd1]; [y1, ... ]; [z1, ... ]]

    cout << "constraints" << endl;
    cout << constraints << endl;

    MatrixXd dersXYZ(6, 3);     // dersXYZ = [[x;xd;xdd; ... xddddd], [y;yd; ...], [z;zd; ...]]; derivatives at t=0
    double time = getPathDers(constraints, dersXYZ);

    cout << "dersXYZ:" << endl;
    cout << dersXYZ << endl;
    cout << time << endl;

    dersToPath(dersXYZ, time, startYaw, pos_to_look_at, look_at_pos, path, timeDiffs, torques);



    return;
}



/*
All states must be given in the world frame.

currentState = [x; y; z; Vx; Vy; Vz; phi; theta; psi; p; q; r]
stateBeforeHoop = [[x; y; z], [Vx; Vy; Vz], [xdd; ydd; zdd]]
stateAfterHoop = [[x; y; z], [Vx; Vy; Vz], [xdd; ydd; zdd]]
*/
int path_planner::getPath(MatrixXd currentState, MatrixXd stateBeforeHoop, MatrixXd stateAfterHoop, double currentYaw, Vector3d hoop_pos, MatrixXd& path, VectorXd& timeDiffs, MatrixXd& torques) {

    // state = [[x, xd, xdd]; [y, yd, ydd]; [z, zd, zdd]]

    int waypoints = 1;				// number of input waypoints to this function, excluding the currentState and endState(stateAfterHoop in current implementation).
    int totalLength = 0;

    MatrixXd allConstraints(6, 6);
    allConstraints.block<3, 3>(0, 0) = currentState;		// constraints of first trajectory part (currentState -> stateBeforeHoop)
    allConstraints.block<3, 3>(3, 0) = stateBeforeHoop;
    allConstraints.block<3, 3>(0, 3) = stateBeforeHoop;		// constraints of second trajectory part (stateBeforeHoop -> stateAfterHoop)
    allConstraints.block<3, 3>(3, 3) = stateAfterHoop;

    double startYaw = currentYaw;
    Vector3d pos_to_look_at = hoop_pos;
    bool look_at_pos = true;

    MatrixXd pathPart;
    VectorXd timeDiffsPart;
    MatrixXd torquesPart;
    MatrixXd constraintsPart(6,3);		// constraints = [[x0, xd0, xdd0]; [y0, ... ]; [z0, ... ]; [x1, xd1, xdd1]; [y1, ... ]; [z1, ... ]]


    for (int i = 0; i <= waypoints; i++) {							// iterate over path intervals.
        cout << "here again" << endl;
        constraintsPart = allConstraints.block<6,3>(0, i*3); 			// get constraints for this path interval

        cout << "yo" << endl;
        getPathSegment(constraintsPart, startYaw, pos_to_look_at, look_at_pos, pathPart, timeDiffsPart, torquesPart);

        cout << "got path segment" << endl;
        cout << pathPart << endl;
        int points = pathPart.rows();
        totalLength += points;

        cout << "that " << endl;

        //path.resize(totalLength, 12);
        cout << "this" << endl;

        MatrixXd path_temp(totalLength, 12);
        cout << "1" << endl;
        path_temp.block(0,0, path.rows(), 12) = path.replicate(path.rows(), 12);
        cout << "2" << endl;
        path_temp.block(totalLength-points-1, 0, points, 12) = pathPart.replicate(points, 12);
        cout << "3" << endl;
        //cout << path_temp << endl;
        path = path_temp.replicate(totalLength, 12);
        //path.block(path.rows()-points-1, 0, points, 12) = pathPart.replicate(points, 12);

        cout << "here" << endl;

        VectorXd timeDiffs_temp(totalLength);
        timeDiffs_temp.block(0,0, timeDiffs.rows(), 1) = timeDiffs.replicate(timeDiffs.rows(),1);
        timeDiffs_temp.block(totalLength-points-1, 0, points, 12) = timeDiffsPart.replicate(points,1);
        timeDiffs = timeDiffs_temp.replicate(totalLength,1);

        //timeDiffs.resize(totalLength);
        //timeDiffs.block(timeDiffs.size()-points-1, 0, points, 1) = timeDiffsPart.replicate(points, 1);

        cout << "there" << endl;

        MatrixXd torques_temp(totalLength,4);
        torques_temp.block(0,0, torques.rows(), 1) = torques.replicate(torques.rows(), 4);
        torques_temp.block(totalLength-points-1,0,points,4) = torquesPart.replicate(points, 4);
        torques = torques_temp.replicate(totalLength, 4);

        cout << "cmon" << endl;
        //torques.resize(totalLength, 4);
        //torques.block(torques.rows()-points-1, 0, points, 4) = torquesPart.replicate(points, 4);


        look_at_pos = false;
        cout << "wtf" << endl;
    }

    cout << "got path" << endl;


    return totalLength;
}


/*
        Let the hoop coordinate frame be as follows:
        The x-axis goes through the main symmetry axis of the hoop, positive direction given by the passage direction.
        The y-axis is horizontal in the earth-frame, positive direction to the right, from the incoming drone point of view.
        The z-axis is as in a righthanded coordinate frame.

        The positive direction of the z-axis of the world frame is parallel to the gravitational acceleration.
        All used frames stand still with respect to the earth.

        The arguments are the following:
        currentState    : state of drone in world frame.
        currentTorque   : [thrust, torqueX, torqueY, torqueZ].
        hoopTransVec    : hoop position in world frame.
        hoopRotMat      : rotation which transforms from hoop frame coordinates to world frame coordinates.
        &path           : each row is a state of the path.
        &timeDiffs      : each i'th element is the time difference between the i-1'th and i'th state in &path.
                        : the first element is the time difference between the current time and the time of the first state in &path.
        &torques        : each row is the [thrust, torqueX, torqueY, torqueZ] corresponding to the same row in &path.

        return value    : number of states in &path.
*/
int path_planner::run(VectorXd currentState, VectorXd currentTorque, Vector3d hoopTransVec, Matrix3d hoopRotMat, MatrixXd& path, VectorXd& timeDiffs, MatrixXd& torques){


    Matrix3d R = hoopRotMat.replicate(3,3);      // rotation
    Vector3d hoop_pos = hoopTransVec.replicate(3,1);                         // translation

    Matrix3d currentDers = getCurrentDers(currentState, currentTorque);        // = [[x, xd, xdd]; [y, yd, ydd]; [z, zd, zdd]]
    Matrix3d beforeHoop = getBeforeHoop(hoop_pos, R);           // = [[x, xd, xdd]; [y, yd, ydd]; [z, zd, zdd]]
    Matrix3d afterHoop = getAfterHoop(hoop_pos, R);             // = [[x, xd, xdd]; [y, yd, ydd]; [z, zd, zdd]]

    int number_of_points = getPath(currentDers, beforeHoop, afterHoop, currentState(8), hoopTransVec, path, timeDiffs, torques);		// coordinates in world frame.

    cout<< "see" << endl;

    return number_of_points;
}


Matrix3d path_planner::getCurrentDers(VectorXd currentState, VectorXd currentTorque){
    Matrix3d currentDers;
    currentDers.block<3,1>(0,0) = currentState.block<3,1>(0,0);
    currentDers.block<3,1>(0,1) = currentState.block<3,1>(3,0);

    double phi = currentState[6];
    double theta = currentState[7];
    double psi = currentState[8];
    double T = currentTorque[0];    //thrust
    currentDers(0,2) = -(sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta))*T/m;
    currentDers(1,2) = -(cos(phi)*sin(psi)*sin(theta) - cos(psi)*sin(psi))*T/m;
    currentDers(2,2) = -cos(phi)*cos(theta)*T/m + g;

    return currentDers;
}

/*
R rotates coordinates fron hoop to world frame.
*/
Matrix3d path_planner::getBeforeHoop(Vector3d hoop_pos, Matrix3d R){
    // state relative to hoop before hoop
    Vector3d dist_in(d_before, 0, 0);				// hoop frame
    Vector3d vel_in(v_in, 0, 0);
    dist_in = hoop_pos + R*dist_in;					// world frame
    vel_in = R*vel_in;

    // set state before hoop
    Matrix3d beforeHoop(3,3);
    beforeHoop.col(0) = dist_in;
    beforeHoop.col(1) = vel_in;
    beforeHoop.col(2) << 0,0,0;

    return beforeHoop;
}

/*
R rotates coordinates fron hoop to world frame.
*/
Matrix3d path_planner::getAfterHoop(Vector3d hoop_pos, Matrix3d R){
    // state relative to hoop after hoop
    Vector3d dist_out(d_after, 0, 0);				// hoop frame
    Vector3d vel_out(v_after, 0, 0);
    dist_out = hoop_pos + R*dist_out;				// world frame
    vel_out = R*vel_out;

    // set state after hoop
    Matrix3d afterHoop(3,3);
    afterHoop.col(0) = dist_out;
    afterHoop.col(1) = vel_out;
    afterHoop.col(2) << 0,0,0;

    return afterHoop;
}


/*
// argument count, argument vector
int main(int argc, char* argv[]){


    MatrixXd path(50, 12);
    run(currentState, hoopTransVec, hoopRotMat, path);

    return 0;
}
*/








