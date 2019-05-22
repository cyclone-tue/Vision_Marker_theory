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

typedef Matrix<double, 12, 1> Vector12d;
typedef Matrix<double, Dynamic, 12> MatrixX12;
typedef Matrix<double, Dynamic, 3> MatrixX3;
typedef Matrix<double, Dynamic, 4> MatrixX4;


class Trajectory {


    public:
        bool valid;
        Trajectory();

        static void testTrajectory();

        int mark(double time);
        void increment_mark(int mark);

        VectorXd state(int mark);
        Vector3d acc(int mark);
        Vector3d jerk(int mark);
        Vector4d torque(int mark);
        double time(int mark);

        void append(Trajectory traj);
        void replace(int mark0, int mark1, Trajectory traj);
        void collapse(MatrixX12& pathRef, VectorXd& timesRef, MatrixX4& torquesRef, MatrixX3& accsRef, MatrixX3& jerksRef, bool& validRef);

        void appendState(const Vector12d& state);
        void appendTime(const double time);
        void appendTorque(const Vector4d& state);
        void appendAcc(const Vector3d& acc);
        void appendJerk(const Vector3d& jerk);
        void appendAll();

        double maxTimeDif();
        double min(char dim);
        double max(char dim);

        void log();

        void setBeginTorque(Vector4d torque);

    //private:
        MatrixX12 path;      // can be made public for debugging purposes
        VectorXd times;
        Matrix<double, Dynamic, 4> torques;
        MatrixX3 accs;
        MatrixX3 jerks;

        MatrixX12 extraPath;
        VectorXd extraTimes;
        Matrix<double, Dynamic, 4> extraTorques;
        MatrixX3 extraAccs;
        MatrixX3 extraJerks;
        Vector4d beginTorque;
        std::vector<Vector2d> marks;
        //std::map<int, int> marks;
        int maxMark = 0;
        int index(int mark);
        int markIndex(int mark);
        void reduce();
};


#endif //MARKER_VISION_TRAJECTORY_H
