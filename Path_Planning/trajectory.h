//
// Created by arnoud on 15-3-19.
//

#ifndef MARKER_VISION_TRAJECTORY_H
#define MARKER_VISION_TRAJECTORY_H

#include <Eigen/Dense>
#include <stdexcept>
#include <vector>
#include <algorithm>
#include <map>

using namespace Eigen;

class Trajectory {


    public:
        bool valid;
        Trajectory();

        int mark(double time);
        void increment_mark(int mark);

        VectorXd state(int mark);
        Vector4d torque(int mark);
        double time(int mark);

        void append(Trajectory traj);
        void replace(int mark0, int mark1, Trajectory traj);
        void collapse(MatrixXd& pathRef, VectorXd& timesRef, MatrixXd& torquesRef, bool& validRef);

        void appendState(const VectorXd state);
        void appendTime(const double time);
        void appendTorque(const Vector4d state);
        void appendAll();




        double maxTimeDif();
        double min(char dim);
        double max(char dim);


    private:
        MatrixXd path;
        VectorXd times;
        MatrixXd torques;
        MatrixXd extraPath;
        VectorXd extraTimes;
        MatrixXd extraTorques;
        std::vector<Vector2d> marks;
        //std::map<int, int> marks;
        int maxMark = 0;
        int index(int mark);
        int markIndex(int mark);
        void reduce();
};


#endif //MARKER_VISION_TRAJECTORY_H
