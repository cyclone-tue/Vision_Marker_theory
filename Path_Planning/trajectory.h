//
// Created by arnoud on 15-3-19.
//

#ifndef MARKER_VISION_TRAJECTORY_H
#define MARKER_VISION_TRAJECTORY_H


class Trajectory {
    MatrixXd path;
    VectorXd timeDiffs;
    MatrixXd torques;
    bool valid;
    public:
        Trajectory();
        appendTraj(Trajectory traj);
        append(MatrixXd path, VectorXd timeDiffs, MatrixXd torques);
    private:
        void reduce();
};


#endif //MARKER_VISION_TRAJECTORY_H
