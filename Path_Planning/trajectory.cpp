//
// Created by arnoud on 15-3-19.
//

#include "trajectory.h"
#include "../logging.h"
#include "spdlog/fmt/ostr.h"

// ==== public functions ====

Trajectory::Trajectory(){
    valid = true;
    path = MatrixX12(0);
    times = VectorXd(0);
    torques = Matrix<double, Dynamic, 4>(0);
    accs = MatrixX3(0);
    jerks = MatrixX3(0);
    extraPath = MatrixX12(0);
    extraTimes = VectorXd(0);
    extraTorques = Matrix<double, Dynamic, 4>(0);
    extraAccs = MatrixX3(0);
    extraJerks = MatrixX3(0);
    return;
}

int Trajectory::mark(double time){
    int index;
    double integral = 0;
    double difference1;
    double difference0 = abs(time - integral);  // previous difference
    for(int i = 0; i < times.size(); i++){
        integral += times(i);
        difference1 = abs(time - integral);
        if(difference1 > difference0){
            index = i;
            break;
        }
        difference0 = difference1;
        if(i == times.size()-1) index = times.size();
    }
    int mark = maxMark + 1;
    maxMark++;

    Vector2d new_element = {mark, index};

    //pp_logger->debug("new marker is {}", new_element);

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
    if(index(mark) <= 0) return beginTorque;
    return torques.row(index(mark)-1);
}
VectorXd Trajectory::state(int mark){
    return path.row(index(mark));
}


void Trajectory::append(Trajectory traj){   // implement using replace();
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

// only works if new trajectory is larger.
void Trajectory::replace(int mark0, int mark1, Trajectory traj){
    MatrixX12 appendedPath;
    VectorXd appendedTimes;
    Matrix<double, Dynamic, 4> appendedTorques;
    MatrixX3 appendedAccs;
    MatrixX3 appendedJerks;
    bool appendedValid;
    traj.collapse(appendedPath, appendedTimes, appendedTorques, appendedAccs, appendedJerks, appendedValid);

    int index0 = index(mark0);
    int index1 = index(mark1);

    int oldPathSize = path.rows();
    int newPathsize = path.rows() - (index1 + 1 - index0) + appendedPath.rows();
    path.conservativeResize(newPathsize);
    times.conservativeResize(newPathsize - 1, 1);
    torques.conservativeResize(newPathsize - 1, 4);
    accs.conservativeResize(newPathsize, 3);
    jerks.conservativeResize(newPathsize, 3);
    valid &= appendedValid;

    int numOfMarks = marks.size();          // update markers
    int index0 = index(mark0);
    int index1 = index(mark1);
    for(int i = numOfMarks-1; i >= 0; i--){
        if(index0 < marks.at(i)(1) and marks.at(i)(1) < index1){
            marks.erase(marks.begin() + i);
        }
        else if(marks.at(i)(1) >= index1){
            Vector2d newMark;
            newMark << marks.at(i)(0), marks.at(i)(1) + appendedPath.rows() - (index1 + 1 - index0);
            marks.push_back(newMark);
            marks.erase(marks.begin() + i);
        }
    }

    // move end of path to end of matrix.
    path.block(index(mark0) + appendedPath.rows(),0, oldPathSize - index(mark1),12) << path.block(index(mark1),0, oldPathSize - index(mark1),12);
    times.block(index(mark0) + appendedPath.rows(),0, oldPathSize - index(mark1),12) << times.block(index(mark1),0, oldPathSize - index(mark1),12);
    torques.block(index(mark0) + appendedPath.rows(),0, oldPathSize - index(mark1),12) << torques.block(index(mark1),0, oldPathSize - index(mark1),12);
    accs.block(index(mark0) + appendedPath.rows(),0, oldPathSize - index(mark1),12) << accs.block(index(mark1),0, oldPathSize - index(mark1),12);
    jerks.block(index(mark0) + appendedPath.rows(),0, oldPathSize - index(mark1),12) << jerks.block(index(mark1),0, oldPathSize - index(mark1),12);


    path.block(index(mark0),0, index(mark0),12)

    newPath << path.block(0,0, index(mark0),12), appendedPath, path.block(index(mark1)+1,0, path.rows() - index(mark1) - 1,12);
    newTimes << times.block(0,0, index(mark0),1), appendedTimes, times.block(index(mark1),0, times.size()-index(mark1),1);
    newTorques << torques.block(0,0, index(mark0),4), appendedTorques, torques.block(index(mark1),0, torques.rows() - index(mark1),4);

    path = newPath;
    times = newTimes;
    torques = newTorques;



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


void Trajectory::appendState(const VectorXd appendedState){
    MatrixXd newExtraPath(extraPath.rows() + 1, 12);
    newExtraPath.block(0,0, extraPath.rows(),12) << extraPath.block(0,0, extraPath.rows(),12);
    newExtraPath.row(newExtraPath.rows()-1) << appendedState.transpose();
    extraPath = newExtraPath;
    return;
}
void Trajectory::appendTime(const double appendedTime){
    VectorXd newExtraTimes(extraTimes.rows() + 1);
    newExtraTimes << extraTimes, appendedTime;
    extraTimes = newExtraTimes;
    return;
}
void Trajectory::appendTorque(const Vector4d appendedTorque){
    MatrixXd newExtraTorques(extraTorques.rows() + 1, 4);
    newExtraTorques.block(0,0, extraTorques.rows(),4) << extraTorques.block(0,0, extraTorques.rows(),4);
    newExtraTorques.row(newExtraTorques.rows()-1) << appendedTorque.transpose();
    extraTorques = newExtraTorques;
    return;
}
void Trajectory::appendAcc(const Vector3d appendedAcc){
    MatrixXd newExtraAccs(extraPath.rows() + 1, 12);
    newExtraAccs.block(0,0, extraAccs.rows(),12) << extraAccs.block(0,0, extraAccs.rows(),12);
    newExtraAccs.row(newExtraAccs.rows()-1) << appendedAcc.transpose();
    extraAccs = newExtraAccs;
    return;
}
void Trajectory::appendJerk(const Vector3d appendedJerk){
    MatrixXd newExtraJerks(extraJerks.rows() + 1, 12);
    newExtraJerks.block(0,0, extraJerks.rows(),12) << extraJerks.block(0,0, extraJerks.rows(),12);
    newExtraJerks.row(newExtraJerks.rows()-1) << appendedJerk.transpose();
    extraJerks = newExtraJerks;
    return;
}
void Trajectory::appendAll(){

    MatrixXd newPath(std::max(0, (int) path.rows()-1) + extraPath.rows(), 12);
    MatrixXd newAccs(std::max(0, (int) accs.rows()-1) + extraAccs.rows(), 12);
    MatrixXd newJerks(std::max(0, (int) jerks.rows()-1) + extraJerks.rows(), 12);
    VectorXd newTimes(times.size() + extraTimes.size());
    MatrixXd newTorques(torques.rows() + extraTorques.rows(), 4);

    newPath << path.block(0,0, std::max(0, (int) path.rows()-1) ,12), extraPath;
    newAccs << accs.block(0,0, std::max(0, (int) accs.rows()-1) ,12), extraAccs;
    newJerks << jerks.block(0,0, std::max(0, (int) jerks.rows()-1) ,12), extraJerks;
    newTimes << times, extraTimes;
    newTorques << torques, extraTorques;

    path = newPath;
    accs = newAccs;
    jerks = newJerks;
    times = newTimes;
    torques = newTorques;

    extraPath = MatrixXd(0,12);
    extraAccs = MatrixXd(0,3);
    extraJerks = MatrixXd(0,3);
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


void Trajectory::log(){
    pp_logger->debug("==== information in this trajectory ====");
    pp_logger->debug("== valid is {}", valid);
    if(not valid) {
        pp_logger->flush();
        return;
    }
    pp_logger->debug("== beginTorque is {}", beginTorque);
    for(int i = 0; i < marks.size(); i++){
        pp_logger->debug("== includes mark {},{}", marks.at(i)(0), marks.at(i)(1));
    }
    MatrixXd info(path.rows(), 1+12+1+4+3+3);
    info << MatrixXd::Zero(info.rows(), info.cols());
    info(index(-1), 0) = time(-1);              // set times of the marks
    for(int i = 0; i < marks.size(); i++){
        int mark = marks.at(i)(0);
        info(index(mark), 0) = time(mark);
    }
    info.block(0,1, path.rows(),12) << path;
    info.block(0,13, times.size(),1) << times;
    info.block(0,14, torques.rows(),4) << torques;
    info.block(0,18, accs.rows(),3) << accs;
    info.block(0,21, jerks.rows(),3) << jerks;
    pp_logger->debug("== times of the marks, path, timeDifs, torques, are: \n {}", info);
}


void Trajectory::setBeginTorque(Vector4d torque){
    beginTorque << torque;
    return;
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
    pp_logger->debug("markers are {}, {}, {}", marks.at(0), marks.at(1), marks.at(2));
    pp_logger->flush();
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