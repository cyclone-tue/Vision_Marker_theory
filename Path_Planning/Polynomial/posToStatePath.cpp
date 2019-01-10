
#include "path_planner.h"


/*
The output is a list of states, at times [time/points, ... , time].
*/
void path_planner::dersToPath(MatrixXd ders, double endTime, double startYaw, Vector3d pos_to_look_at, bool look_at_pos, MatrixXd& path, VectorXd& timeDiffs, MatrixXd& torques) {          // needs fixing
    // ders = [[x;xd;xdd; ... xddddd], [y;yd; ...], [z;zd; ...]]; derivatives at t=0

    int points = 50;

    path.resize(points, 12);
    timeDiffs.resize(points);
    torques.resize(points,4);


    for(int i = 0; i < points; i++){
        double time = endTime * i/(points-1);
        VectorXd ders2 = ders.col(0);
        path(i, 0) = taylor(ders2, time);
        ders2 = ders.col(1);
        path(i, 1) = taylor(ders2, time);
        ders2 = ders.col(2);
        path(i, 2) = taylor(ders2, time);
        Vector3d pos = path.block<1,3>(i,0);

        VectorXd ders3 = ders.block<5,1>(1,0);
        path(i, 3) = taylor(ders3, time);
        ders3 = ders.block<5,1>(1,1);
        path(i, 4) = taylor(ders3, time);
        ders3 = ders.block<5,1>(1,2);
        path(i, 5) = taylor(ders3, time);

        path(i, 6) = 0;
        path(i, 7) = 0;
        if(look_at_pos) path(i, 8) = getYawToHoop(pos, pos_to_look_at);
        else path(i, 8) = startYaw;


        path(i, 9) = 0;
        path(i, 10) = 0;
        path(i, 11) = 0;

        timeDiffs(i) = endTime/(points - 1);

        torques(0) = 0;
        torques(1) = 0;
        torques(2) = 0;
        torques(3) = 0;
    }

    timeDiffs(0) = 0;

    return;
}


/*
Vector3d getPos(MatrixXd dersXYZ, double time){
    VectorXd dersX(6) = dersXYZ.row(0);
    VectorXd dersY(6) = dersXYZ.row(1);
    VectorXd dersZ(6) = dersXYZ.row(2);

    double x = taylor(dersX, time);
    double y = taylor(dersY, time);
    double z = taylor(dersZ, time);

    Vector3d position;
    position << x,y,z;
    return position;
}
*/


/*
the input matrix has dimensions 3x6, the rows corresponding to X,Y,Z respectively.
*/
double path_planner::getYawToHoop(Vector3d pos, Vector3d pos_to_look_at){

    Vector3d difference = pos_to_look_at - pos;
    double lengthXY = sqrt(pow(difference(0), 2) + pow(difference(1), 2));
    double yaw = acos(difference(0) / lengthXY);
    if (difference(0) < 0) yaw = -yaw;

    return yaw;
}

