//
// Created by arnoud on 15-3-19.
//

#include "trajectory.h"
#include "../logging.h"
#include "spdlog/fmt/ostr.h"

// ==== public functions ====


void Trajectory::testTrajectory(){

    // test appendState()
    Vector12d test12;
    test12 << 0,1,2,3,4,5,6,7,8,9,10,11;

    Trajectory traj = Trajectory();

    traj.appendState(test12);
    pp_logger->debug("extraPath: {}", traj.extraPath);
    // test appendTime()
    double testD = 13;
    traj.appendTime(testD);
    pp_logger->debug("extraTimes: {}", traj.extraTimes);
    // test appendTorque()
    Vector4d test4;
    test4 << 3,2,1,0;
    traj.appendTorque(test4);
    pp_logger->debug("extraTorques: {}", traj.extraTorques);
    // test appendAcc()
    Vector3d test3;
    test3 << 2,1,0;
    traj.appendAcc(test3);
    pp_logger->debug("extraAccs: {}", traj.extraAccs);
    // test appendJerk()
    traj.appendJerk(test3);
    pp_logger->debug("extraJerks: {}", traj.extraJerks);

    // test appendAll()
    try{
        traj.appendState(test12);
        traj.appendAcc(test3);
        traj.appendJerk(test3);
        traj.appendAll();
        pp_logger->debug("appendAll: path:\n{} times:\n{} torques:\n{} accs:\n{} jerks:\n{}", traj.path, traj.times, traj.torques, traj.accs, traj.jerks);
        traj.appendState(test12);
        traj.appendAcc(test3);
        traj.appendJerk(test3);
        traj.appendTorque(test4);
        traj.appendTime(testD);
        traj.appendAll();
        pp_logger->debug("appendAll: path:\n{} times:\n{} torques:\n{} accs:\n{} jerks:\n{}", traj.path, traj.times, traj.torques, traj.accs, traj.jerks);
    }
    catch( const std::invalid_argument& e ) {
        pp_logger->error("error in appendAll()");
    }

    // test mark()
    int mark0 = traj.mark(13);
    int mark1 = traj.mark(29);

    // test log()
    traj.log();

    // test replace()
    traj.replace(mark0, mark1, traj);
    traj.log();
    traj.replace(traj.mark(0), traj.mark(13), traj);
    traj.log();

    pp_logger->debug("end of testTrajectory()");
    return;
}



Trajectory::Trajectory(){
    valid = true;
    path = MatrixX12(0, 12);
    times = VectorXd(0);
    torques = Matrix<double, Dynamic, 4>(0, 4);
    accs = MatrixX3(0, 3);
    jerks = MatrixX3(0, 3);
    extraPath = MatrixX12(0, 12);
    extraTimes = VectorXd(0);
    extraTorques = Matrix<double, Dynamic, 4>(0, 4);
    extraAccs = MatrixX3(0, 3);
    extraJerks = MatrixX3(0, 3);
    beginTorque = {0,0,0,0};
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

    marks.push_back(new_element);
    return mark;
}

// should give an error when the mark cannot be increased.
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
Vector3d Trajectory::acc(int mark){
    return accs.row(index(mark));
}
Vector3d Trajectory::jerk(int mark){
    return jerks.row(index(mark));
}



void Trajectory::append(Trajectory traj){   // implement using replace();
    replace(-1, -1, traj);
    return;
}

// only works if new trajectory is larger.
void Trajectory::replace(int mark0, int mark1, Trajectory traj){

    // all new data
    MatrixX12 appendedPath;
    VectorXd appendedTimes;
    Matrix<double, Dynamic, 4> appendedTorques;
    MatrixX3 appendedAccs;
    MatrixX3 appendedJerks;
    bool appendedValid;
    traj.collapse(appendedPath, appendedTimes, appendedTorques, appendedAccs, appendedJerks, appendedValid);

    int index0 = index(mark0);
    int index1 = index(mark1);

    int intervalls0 = index(mark0) - index(0);
    int intervalls1 = appendedTimes.size();
    int intervalls1old = index(mark1) - index(mark0);
    int intervalls2 = index(-1) - index(mark1);
    int intervalls = intervalls0 + intervalls1 + intervalls2;

    int points0 = intervalls0;
    int points1 = intervalls1 + 1;
    int points1old = intervalls1old + 1;
    int points2 = intervalls2;
    int points = points0 + points1 + points2;

    path.conservativeResize(points, 12);
    times.conservativeResize(intervalls, 1);
    torques.conservativeResize(intervalls, 4);
    accs.conservativeResize(points, 3);
    jerks.conservativeResize(points, 3);
    valid &= appendedValid;

    int numOfMarks = marks.size();          // update markers
    index0 = index(mark0);
    index1 = index(mark1);
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
    path.bottomRows(points2) << path.block(points0 + points1old,0, points2,12);
    times.tail(intervalls2) << times.segment(intervalls0 + intervalls1old, intervalls2);
    torques.bottomRows(intervalls2) << torques.block(intervalls0 + intervalls1old,0, intervalls2,12);
    accs.bottomRows(points2) << accs.block(points0 + points1old,0, points2,3);
    jerks.bottomRows(points2) << jerks.block(points0 + points1old,0, points2,3);

    path.block(points0,0, points1,12) << appendedPath;
    times.segment(intervalls0,intervalls1) << appendedTimes;
    torques.block(intervalls0,0, intervalls1,4) << appendedTorques;
    accs.block(points0,0, points1,3) << appendedAccs;
    jerks.block(points0,0, points1,3) << appendedJerks;
    return;
}
void Trajectory::collapse(MatrixX12& pathRef, VectorXd& timesRef, MatrixX4& torquesRef, MatrixX3& accsRef, MatrixX3& jerksRef, bool& validRef){
    pathRef = path;
    timesRef = times;
    torquesRef = torques;
    accsRef = accs;
    jerksRef = jerks;
    validRef = valid;
    valid = false;
    return;
}


void Trajectory::appendState(const Vector12d& appendedState){    //tested
    extraPath.conservativeResize(extraPath.rows()+1, 12);
    extraPath.row(extraPath.rows()-1) << appendedState.transpose();
    return;
}
void Trajectory::appendTime(const double appendedTime){     //tested
    extraTimes.conservativeResize(extraTimes.size()+1, 1);
    extraTimes(extraTimes.size()-1) = appendedTime;
    return;
}
void Trajectory::appendTorque(const Vector4d& appendedTorque){       //tested
    extraTorques.conservativeResize(extraTorques.rows()+1, 4);
    extraTorques.row(extraTorques.rows()-1) << appendedTorque.transpose();
    return;
}
void Trajectory::appendAcc(const Vector3d& appendedAcc){     //tested
    extraAccs.conservativeResize(extraAccs.rows()+1, 3);
    extraAccs.row(extraAccs.rows()-1) << appendedAcc.transpose();
    return;
}
void Trajectory::appendJerk(const Vector3d& appendedJerk){       //tested
    extraJerks.conservativeResize(extraJerks.rows()+1, 3);
    extraJerks.row(extraJerks.rows()-1) << appendedJerk.transpose();
    return;
}
// time is leading in this function
void Trajectory::appendAll(){       //tested

    int newIntervalls = extraTimes.size();
    int oldIntervalls = times.size();
    int intervalls = newIntervalls + oldIntervalls;

    int oldPoints;
    int newPoints;
    if(oldIntervalls == 0){
        oldPoints = 0;
        newPoints = newIntervalls + 1;
    } else{
        oldPoints = oldIntervalls + 1;
        newPoints = newIntervalls;
    }
    int points = oldPoints + newPoints;

    bool invalidTraj = (path.rows() != oldPoints || accs.rows() != oldPoints || jerks.rows() != oldPoints || torques.rows() != oldIntervalls);
    //pp_logger->error("newPoints: {}, newIntervalls: {}, extraPath: {}, extraAccs: {}, extraJerks: {}, extraTorques: {}.", newPoints, newIntervalls, extraPath.rows(), extraAccs.rows(), extraJerks.rows(), extraTorques.rows());
    bool invalidAction = (extraPath.rows() < newPoints || extraAccs.rows() < newPoints || extraJerks.rows() < newPoints || extraTorques.rows() < newIntervalls);
    if(invalidTraj){
        pp_logger->error("appendAll: dimensions of trajectory incorrect: path {}, accs {}, jerks {}, torques {}, times {}", path.rows(), accs.rows(), jerks.rows(), torques.rows(), times.size());
        pp_logger->flush();
        throw std::invalid_argument("appendAll: dimensions of trajectory incorrect");
    }
    if(invalidAction){
        pp_logger->error("appendAll: dimensions of trajectory incorrect: extrapath {}, extraaccs {}, extrajerks {}, extratorques {}, extratimes {}", extraPath.rows(), extraAccs.rows(), extraJerks.rows(), extraTorques.rows(), extraTimes.size());
        pp_logger->flush();
        throw std::invalid_argument("appendAll: called with invalid dimensions");
    }

    path.conservativeResize(points, 12);
    path.bottomRows(newPoints) << extraPath.topRows(newPoints);
    accs.conservativeResize(points, 3);
    accs.bottomRows(newPoints) << extraAccs.topRows(newPoints);
    jerks.conservativeResize(points, 3);
    jerks.bottomRows(newPoints) << extraJerks.topRows(newPoints);

    times.conservativeResize(intervalls, 1);
    times.tail(newIntervalls) << extraTimes.head(newIntervalls);
    torques.conservativeResize(intervalls, 4);
    torques.bottomRows(newIntervalls) << extraTorques.topRows(newIntervalls);

    extraPath = MatrixX12(0,12);
    extraAccs = MatrixX3(0,3);
    extraJerks = MatrixX3(0,3);
    extraTimes = VectorXd(0);
    extraTorques = Matrix<double, Dynamic, 4>(0, 4);
    return;
}


double Trajectory::maxTimeDif(){
    return times.maxCoeff();
}
double Trajectory::min(char dim){
    if(dim == 'x') return path.col(0).minCoeff();
    if(dim == 'y') return path.col(1).minCoeff();
    if(dim == 'z') return path.col(2).minCoeff();
    pp_logger->flush();
    throw std::invalid_argument( "min(): received invalid dimension" );
}
double Trajectory::max(char dim){
    if(dim == 'x') return path.col(0).maxCoeff();
    if(dim == 'y') return path.col(1).maxCoeff();
    if(dim == 'z') return path.col(2).maxCoeff();
    pp_logger->flush();
    throw std::invalid_argument( "max(): received invalid dimension" );
}


void Trajectory::log(){
    pp_logger->debug("==== information in this trajectory ====");
    pp_logger->debug("== valid is {}", valid);
    if(not valid) {
        pp_logger->flush();
        //return;
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
    pp_logger->debug("== times of the marks, path, timeDifs, torques, accs, jerks are: \n{}", info);
    pp_logger->flush();
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
        cout << mark << endl;
        pp_logger->flush();
        throw std::invalid_argument( "markIndex(): received pre-defined marker" );
    }
    for(unsigned long i = 0; i < marks.size(); i++){
        if(marks.at(i)(0) == mark){
            return i;
        }
    }
    pp_logger->debug("markers are {}, {}, {}", marks.at(0), marks.at(1), marks.at(2));
    pp_logger->flush();
    throw std::invalid_argument( "markIndex: received invalid marker" );
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