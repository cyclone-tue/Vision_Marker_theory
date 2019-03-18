// this file contains functions for logging and visualization.

//#include "V_PP.h"
#include "logging.h"
#include "general.h"
#include <chrono>
#include <thread>

/*
path contains x times 12 elements.
 */
void runVisualize(VectorXd& currentState, Trajectory traj, Vector3d& hoopTransVec, Matrix3d& hoopRotMat, bool displayPath){

    Mat frame;
    vision::debugFrame.copyTo(frame);

    if(displayPath) {


        vector<Point3d> cvPoints;
        for (int mark = traj.mark(traj.time(-1)); traj.time(mark) <= traj.time(-1); traj.increment_mark(mark)) {
            VectorXd state = traj.state(mark);
            cvPoints.push_back(Point3d(state(0), state(1), state(2)));
        }

        vector<Point2d> imagePoints;
        vision::projectPointsOntoCam(cvPoints, currentState, imagePoints);

        for (int j = 0; j < imagePoints.size() - 1; j++) {

            Point point0 = imagePoints[j];
            Point point1 = imagePoints[j+1];

            if(clipLine(Size(frame.cols, frame.rows), point0, point1)){
                line(frame, point0, point1, Scalar(int(255 / imagePoints.size() * j), 0,
                                                                       int(255 / imagePoints.size() *
                                                                           (imagePoints.size() - j))), 3);
            }

        }

    }

    imshow("out", frame);
    waitKey(1);

    showPathInteractive(traj, hoopTransVec, hoopRotMat);

    return;
}


void showPathInteractive(Trajectory traj, Vector3d hoopTransVec, Matrix3d hoopRotMat){

    vpp_logger->flush();
    pp_logger->flush();
    v_logger->flush();

    double min_x = traj.min('x');
    double max_x = traj.max('x');
    double min_y = traj.min('y');//path.col(1).minCoeff();
    double max_y = traj.max('y');//path.col(1).maxCoeff();
    double min_z = traj.min('z');
    double max_z = traj.max('z');
    double maxTime = traj.time(-1);

    double time = 0;            // generate trajectory
    std::vector<boost::tuple<double, double, double>> positions;
    std::vector<boost::tuple<double, double, double>> dots;
    for(int mark = traj.mark(traj.time(-1)); traj.time(mark) <= traj.time(-1); traj.increment_mark(mark)) {
        double x = traj.state(mark)(0);
        double y = traj.state(mark)(1);
        double z = traj.state(mark)(2);
        positions.push_back(boost::make_tuple(x, y, z));
    }

    double radius = 0.5;        // generate hoop
    std::vector<boost::tuple<double, double, double>> hoop;
    for(double theta = 0; theta <= 2*M_PI; theta += 2*M_PI/20){
        // hoop frame
        Vector3d hoopPoint;
        hoopPoint << 0, 0.5*cos(theta), 0.5*sin(theta);

        // world frame
        Vector3d hoopPoint_w = hoopRotMat*hoopPoint + hoopTransVec;
        hoop.push_back(boost::make_tuple(hoopPoint_w(0), hoopPoint_w(1), hoopPoint_w(2)));
    }
    gp << "set term x11 0\n";
    gp << "set xlabel 'x'\n";
    gp << "set ylabel 'y'\n";
    gp << "set zlabel 'z'\n";
    gp << "set view equal xyz\n";
    gp << "splot '-' u 1:2:3 with lines, '-' u 1:2:3 with lines, '-' u 1:2:3 pt 7 ps 1\n";
    gp.send1d(positions);
    gp.send1d(hoop);
    gp.send1d(dots);

    gp << "set view 240,110\n";

    gp << "set term x11 1\n";


    gp << "set xlabel 'x'\n";
    gp << "set ylabel 'y'\n";
    gp << "set zlabel 'z'\n";
    //gp << "set view azimuth 180\n";
    //gp << "set view -90,0\n";
    gp << "set xrange [" << min_x - 1 << ":" << max_x + 1 << "]\n";
    gp << "set yrange [" << min_y - 1 << ":" << max_y + 1 << "]\n";
    gp << "set zrange [" << max_z + 1 << ":" << min_z - 1 << "]\n";


    gp << "set view equal xyz\n";

    while(true) {
        // make timer
        for (int mark = traj.mark(traj.time(-1)); traj.time(mark) <= traj.time(-1); traj.increment_mark(mark)) {

            std::vector<std::vector<boost::tuple<double, double, double>>> drone;
            Vector3d pos = traj.state(mark).block<1, 3>(0, 0);
            Vector3d ang = traj.state(mark).block<1, 3>(0, 6);
            drone = getDronePoints(pos, ang);
            gp
                    << "splot '-' u 1:2:3 with lines, '-' u 1:2:3 with lines, '-' u 1:2:3 with lines, '-' u 1:2:3 with lines, '-' u 1:2:3 with lines, '-' u 1:2:3 with lines, '-' u 1:2:3 with lines, '-' u 1:2:3 with lines, '-' u 1:2:3 with lines \n";
            gp.send1d(positions);
            gp.send1d(hoop);
            for (int i = 0; i < static_cast<int>(drone.size()); i++) {
                gp.send1d(drone.at(i));
            }

            //std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(dt*1000)));

        }
        break;
    }

    //getchar();    // wait for user input (to keep pipe open)
    return;
}


std::vector<std::vector<boost::tuple<double, double, double>>> getDronePoints(Vector3d pos, Vector3d ang){
    double dist_diag = 0.3;  // distance between centers of rotors
    double radius_rotor = 0.1;

    // construct drone frame
    std::vector<boost::tuple<double, double, double>> bar_0;
    std::vector<boost::tuple<double, double, double>> bar_1;
    std::vector<boost::tuple<double, double, double>> bar_2;

    bar_0.push_back(boost::make_tuple(dist_diag/sqrt(2), dist_diag/sqrt(2), 0));
    bar_0.push_back(boost::make_tuple(-dist_diag/sqrt(2), -dist_diag/sqrt(2), 0));
    bar_1.push_back(boost::make_tuple(-dist_diag/sqrt(2), dist_diag/sqrt(2), 0));
    bar_1.push_back(boost::make_tuple(dist_diag/sqrt(2), -dist_diag/sqrt(2), 0));
    bar_2.push_back(boost::make_tuple(dist_diag/sqrt(2), dist_diag/sqrt(2), 0));
    bar_2.push_back(boost::make_tuple(dist_diag/sqrt(2), -dist_diag/sqrt(2), 0));

    // construct rotors
    std::vector<boost::tuple<double, double, double>> rotor_fr;
    std::vector<boost::tuple<double, double, double>> rotor_fl;
    std::vector<boost::tuple<double, double, double>> rotor_br;
    std::vector<boost::tuple<double, double, double>> rotor_bl;
    for(double theta = 0; theta <= 2*M_PI; theta += 2*M_PI/20){
        double dist = dist_diag/sqrt(2);
        rotor_fr.push_back(boost::make_tuple(radius_rotor*cos(theta) + dist , radius_rotor*sin(theta) + dist, 0));
        rotor_fl.push_back(boost::make_tuple(radius_rotor*cos(theta) + dist , radius_rotor*sin(theta) - dist, 0));
        rotor_br.push_back(boost::make_tuple(radius_rotor*cos(theta) - dist , radius_rotor*sin(theta) + dist, 0));
        rotor_bl.push_back(boost::make_tuple(radius_rotor*cos(theta) - dist , radius_rotor*sin(theta) - dist, 0));
    }

    std::vector<std::vector<boost::tuple<double, double, double>>> dronePoints;
    dronePoints.push_back(bar_0);
    dronePoints.push_back(bar_1);
    dronePoints.push_back(bar_2);
    dronePoints.push_back(rotor_fr);
    dronePoints.push_back(rotor_fl);
    dronePoints.push_back(rotor_br);
    dronePoints.push_back(rotor_bl);

    dronePoints = rotateDrone(dronePoints, pos, ang);

    return dronePoints;
}

// not very well written yet.
std::vector<std::vector<boost::tuple<double, double, double>>> rotateDrone(std::vector<std::vector<boost::tuple<double, double, double>>> dronePoints, Vector3d pos, Vector3d ang){
    Matrix3d droneFrameToWorldFrameRotation = anglesToRotMatXYZ(ang(0), ang(1), ang(2));

    std::vector<std::vector<boost::tuple<double, double, double>>> newDronePoints;

    for(int i = 0; i < static_cast<int>(dronePoints.size()); i++){
        std::vector<boost::tuple<double, double, double>> newSegment;
        for(int j = 0; j < static_cast<int>(dronePoints.at(i).size()); j++){
            Vector3d oldPoint;
            oldPoint << boost::get<0>(dronePoints.at(i).at(j)), boost::get<1>(dronePoints.at(i).at(j)), boost::get<2>(dronePoints.at(i).at(j));
            Vector3d newPoint = droneFrameToWorldFrameRotation*oldPoint + pos;
            newSegment.push_back(boost::make_tuple(newPoint(0), newPoint(1), newPoint(2)));
        }
        newDronePoints.push_back(newSegment);
    }
    return newDronePoints;
}



