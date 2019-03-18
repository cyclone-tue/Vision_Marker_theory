//
// Created by arnoud on 15-3-19.
//

#include "trajectory.h"
#include "../logging.h"
#include "spdlog/fmt/ostr.h"

// ==== public functions ====

Trajectory::Trajectory(){
    valid = true;
    path = MatrixXd(0,12);
    times = VectorXd(0);
    torques = MatrixXd(0,4);
    extraPath = MatrixXd(0,12);
    extraTimes = VectorXd(0);
    extraTorques = MatrixXd(0,4);
    return;
}

int Trajectory::mark(double time){
    int index;
    double integral = 0;
    double difference1;
    double difference0 = abs(time - integral);
    for(int i = 0; i < times.size(); i++){
        difference1 = abs(time - integral);
        if(difference1 > difference0){
            index = i-1;
            break;
        }
        integral += times(i);
        difference0 = abs(time - integral);
    }
    int mark;
    if(marks.size() == 0){
        mark = 1;
    } else{
        mark = marks.at(marks.size() - 1)(0) + 1;
    }
    Vector2d new_element = {mark, index};
    marks.push_back(new_element);
    return mark;
}
void Trajectory::increment_mark(int mark){
    marks.at(markIndex(mark))(1) += 1;
    return;
}


double Trajectory::time(int mark){
    int maxIndex = index(mark);
    double time = 0;
    for(int i = 0; i < maxIndex; i++){
        time += times(i);
    }
    return time;
}
Vector4d Trajectory::torque(int mark){
    return torques.row(index(mark)-1);
}
VectorXd Trajectory::state(int mark){
    pp_logger->debug("Starting state(mark)");
    pp_logger->flush();
    return path.row(index(mark));
}


void Trajectory::append(Trajectory traj){
    MatrixXd appendedPath;
    VectorXd appendedTimes;
    MatrixXd appendedTorques;
    bool appendedValid;
    traj.collapse(appendedPath, appendedTimes, appendedTorques, appendedValid);

    MatrixXd newPath(path.rows() - 1 + appendedPath.rows(), 12);
    VectorXd newTimes(times.size() + appendedTimes.size());
    MatrixXd newTorques(torques.rows() + appendedTorques.rows(), 4);
    valid &= appendedValid;

    newPath << path.block(0,0, path.rows()-1,12), appendedPath;
    newTimes << times, appendedTimes;
    newTorques << torques, appendedTorques;

    path = newPath;
    times = newTimes;
    torques = newTorques;
    return;
}
void Trajectory::replace(int mark0, int mark1, Trajectory traj){
    MatrixXd appendedPath;
    VectorXd appendedTimes;
    MatrixXd appendedTorques;
    bool appendedValid;
    traj.collapse(appendedPath, appendedTimes, appendedTorques, appendedValid);

    MatrixXd newPath(path.rows() - (index(mark1) + 1 - index(mark0)) + appendedPath.rows(), 12);
    VectorXd newTimes(times.size() + appendedTimes.size());
    MatrixXd newTorques(torques.rows() + appendedTorques.rows(), 4);
    valid &= appendedValid;

    newPath << path.block(0,0, index(mark0),12), appendedPath, path.block(mark1+1,0, path.rows() - index(mark1) - 1,12);
    newTimes << times.block(0,0, index(mark0),1), appendedTimes, times.block(index(mark1),0, times.size()-index(mark1),1);
    newTorques << torques.block(0,0, index(mark0),1), appendedTorques, torques.block(index(mark1),0, torques.size()-index(mark1),1);

    path = newPath;
    times = newTimes;
    torques = newTorques;

    for(int i = 0; i < marks.size(); i++){
        if(marks.at(index(mark0))(1) < marks.at(i)(1) and marks.at(i)(1) < marks.at(index(mark1))(1)){
            marks.erase(marks.begin() + i);
        }
        else if(marks.at(i)(1) >= marks.at(index(mark1))(1)){
            marks.at(i)(1) += appendedPath.rows() - (index(mark1) + 1 - index(mark0));
        }
    }

    return;
}
void Trajectory::collapse(MatrixXd& pathRef, VectorXd& timesRef, MatrixXd& torquesRef, bool& validRef){
    pathRef = path;
    timesRef = times;
    torquesRef = torques;
    validRef = valid;
    valid = false;
    return;
}


void Trajectory::appendState(VectorXd appendedState){
    MatrixXd newExtraPath(extraPath.rows() + 1, 12);
    newExtraPath << extraPath, appendedState;
    extraPath = newExtraPath;
    return;
}
void Trajectory::appendTime(double appendedTime){
    VectorXd newExtraTimes(extraTimes.rows() + 1);
    newExtraTimes << extraTimes, appendedTime;
    extraTimes = newExtraTimes;
    return;
}
void Trajectory::appendTorque(Vector4d appendedTorque){
    MatrixXd newExtraTorques(extraTorques.rows() + 1, 4);
    newExtraTorques << extraTorques, appendedTorque;
    extraTorques = newExtraTorques;
    return;
}
void Trajectory::appendAll(){

    MatrixXd newPath(std::max(0, (int) path.rows()-1) + extraPath.rows(), 12);
    VectorXd newTimes(times.size() + extraTimes.size());
    MatrixXd newTorques(torques.rows() + extraTorques.rows(), 4);

    newPath << path.block(0,0, std::max(0, (int) path.rows()-1) ,12), extraPath;
    newTimes << times, extraTimes;
    newTorques << torques, extraTorques;

    path = newPath;
    times = newTimes;
    torques = newTorques;

    extraPath = MatrixXd(0,12);
    extraTimes = VectorXd(0);
    extraTorques = MatrixXd(0,4);
    return;
}


double Trajectory::maxTimeDif(){
    return times.maxCoeff();
}
double Trajectory::min(char dim){
    if(dim == 'x') return path.row(0).minCoeff();
    if(dim == 'y') return path.row(1).minCoeff();
    if(dim == 'z') return path.row(2).minCoeff();
    throw std::invalid_argument( "received invalid dimension" );
}
double Trajectory::max(char dim){
    if(dim == 'x') return path.row(0).maxCoeff();
    if(dim == 'y') return path.row(1).maxCoeff();
    if(dim == 'z') return path.row(2).maxCoeff();
    throw std::invalid_argument( "received invalid dimension" );
}



// ==== private functions ====

int Trajectory::index(int mark){
    if(mark == 0){
        return 0;
    }
    if(mark == -1){
        return path.rows() - 1;
    }
    return marks.at(markIndex(mark))(1);
}
int Trajectory::markIndex(int mark){
    if(mark == 0 || mark == -1){
        throw std::invalid_argument( "received pre-defined marker" );
    }
    for(unsigned long i = 0; i < marks.size(); i++){
        if(marks.at(i)(0) == mark){
            return i;
        }
    }
    throw std::invalid_argument( "received invalid marker" );
}


/*
void trajectory::reduce(std::vector<MatrixXd> subPaths, std::vector<VectorXd> subTimeDiffs, std::vector<MatrixXd> subTorques, MatrixXd& path, VectorXd& timeDiffs, MatrixXd& torques){
    for(int i = 1; i < subPaths.size(); i++){
        removeRow(subPaths.at(i), 0);
        removeElement(subTimeDiffs.at(i), 0);
        removeRow(subTorques.at(i), 0);
    }
    path = concatenate(subPaths);
    timeDiffs = concatenateVec(subTimeDiffs);
    torques = concatenate(subTorques);
    return;
}
*/