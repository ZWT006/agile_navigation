/*
 * @Author: wentao zhang && zwt190315@163.com
 * @Date: 2023-06-16
 * @LastEditTime: 2023-07-11
 * @Description: polynomial trajectory segment
 * @reference: 
 * 
 * Copyright (c) 2023 by wentao zhang, All Rights Reserved. 
 */

#ifndef _SEGMENT_H_
#define _SEGMENT_H_

#include <vector>
#include <iostream>
#include <Eigen/Eigen>
#include <cmath>

#include "root_finder.hpp"

struct Segment;
typedef Segment* SegmentPtr;

struct Segment
{
    double duration;
    std::vector<double> pxcoeff;
    std::vector<double> pycoeff;
    std::vector<double> pqcoeff;
};

// Polynomial order and trajectory dimension are fixed here
// 7th order polynomial:     % q0 q1 q2 q3 q4 q5 q6 q7
// !!! 索引序号越大,次数越低 !!!
typedef Eigen::Matrix<double, 3, 8> CoefficientMat;
typedef Eigen::Matrix<double, 3, 7> VelCoefficientMat;
typedef Eigen::Matrix<double, 3, 6> AccCoefficientMat;
typedef Eigen::Matrix<double, 3, 5> JerCoefficientMat;
typedef Eigen::Matrix<double, 4, 8> timeCoefficientMat;

// 单独一个 多项式轨迹 段
/************************************************************************************************************
 * @description: [x,y,q] 多项式轨迹段类 多项式的系数矩阵计算,时间矩阵计算,轨迹段的状态计算
 * @reference: https://github.com/ZJU-FAST-Lab/Fast-Perching/tree/master/src/traj_opt/include/traj_opt/poly_traj_utils.hpp
 */
class Piece {
private:
    double duration;  // duration of the piece
    double dt;        // discretization time interval
    int discreteNum; // number of discretization
    CoefficientMat coeffMat;
    timeCoefficientMat xcoefftimeMat;
    timeCoefficientMat ycoefftimeMat;
    timeCoefficientMat qcoefftimeMat;
    timeCoefficientMat timeMat;

    // std::vector<Eigen::Vector3d> posVector;
    // std::vector<Eigen::Vector3d> velVector;
    // std::vector<Eigen::Vector3d> accVector;


    ////Eigen::Matrix3Xd#############################################################################
    inline void updatePosVector() {
        posVector.setZero();
        Eigen::Vector3d pos(0.0, 0.0, 0.0);
        for (int idx = 0; idx <= discreteNum; idx++) {
            pos = getPos(idx * dt);
            posVector.col(idx) = pos;
        }
    }
    inline void updateVelVector() {
        velVector.setZero();
        Eigen::Vector3d vel(0.0, 0.0, 0.0);
        for (int idx = 0; idx <= discreteNum; idx++) {
            vel = getVel(idx * dt);
            velVector.col(idx) = vel;
        }
    }
    inline void updateAccVector() {
        accVector.setZero();
        Eigen::Vector3d acc(0.0, 0.0, 0.0);
        for (int idx = 0; idx <= discreteNum; idx++) {
            acc = getAcc(idx * dt);
            accVector.col(idx) = acc;
        }
    }
    inline void updateJerVector() {
        jerVector.setZero();
        Eigen::Vector3d jer(0.0, 0.0, 0.0);
        for (int idx = 0; idx <= discreteNum; idx++) {
            jer = getJer(idx * dt);
            jerVector.col(idx) = jer;
        }
    }

    //################################################################################################
    //// get coefficients of normalized polynomial in time [t] 类似计算
    /*
    % coffeicents matrixs 的形式为 [A] //💥 形式B不好 因为不是主对角线元素都不为0 操作有点麻烦
    % q0 q1 q2 q3 q4 q5 q6 q7
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % A   coeff = [1,  1*t,  1*t^2,  1*t^3,  1*t^4,  1*t^5,  1*t^6,  1*t^7;
    %              0,  1,    2*t,    3*t^2,  4*t^3,  5*t^4,  6*t^5,  7*t^6;
    %              0,  0,    2,      6*t,    12*t^2, 20*t^3, 30*t^4, 42*t^5;
    %              0,  0,    0,      6,      24*t,   60*t^2, 120*t^3,210*t^4];
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % B   =  左右翻转A
    % B   coeff = [1*t^7, 1*t^6,  1*t^5,  1*t^4,  1*t^3,  1*t^2,  1*t,   1     ]
    %             [7*t^6, 6*t^5,  5*t^4,  4*t^3,  3*t^2,  2*t,    1,     0     ]
    %             [42*t^5,30*t^4, 20*t^3, 12*t^2, 6*t,    2,      0,     0     ]
    %             [210*t^4,120*t^3,60*t^2,24*t,    6,     0,      0,     0     ]
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    */
    ////#############################################################################################
    inline void updateTimeMat() {
        timeMat.setZero();
        double t = 1.0;
        for (int i = 0; i <= 7; i++) {
            timeMat.row(0)(i) = t;
            t *= duration;
        }
        t = 1.0;    double n = 1.0;
        for (int i = 1; i <= 7; i++) {
            timeMat.row(1)(i) = n * t;
            t *= duration;
            n++;
        }
        t = 1.0;    n = 2.0;    double m = 1.0;
        for (int i = 2; i <= 7; i++) {
            timeMat.row(2)(i) = n * m * t;
            t *= duration;
            n++;
            m++;
        }
        t = 1.0;    n = 3.0;    m = 2.0;    double l = 1.0;
        for (int i = 3; i <= 7; i++) {
            timeMat.row(3)(i) = n * m * l * t;
            t *= duration;
            n++;
            m++;
            l++;
        }
    }


public:
    Piece() = default;

    Eigen::Matrix3Xd posVector;
    Eigen::Matrix3Xd velVector;
    Eigen::Matrix3Xd accVector;
    Eigen::Matrix3Xd jerVector;

    // 这里的用法是 *类的初始化列表* 一定要搞清楚先构造的是哪一个，是按照成员声明顺序进行构造的！
    Piece(double dur, int disNum,const CoefficientMat &cMat)
        : duration(dur), discreteNum(disNum), coeffMat(cMat) {
                dt = duration / discreteNum;
                posVector.resize(3, discreteNum + 1);
                velVector.resize(3, discreteNum + 1);
                accVector.resize(3, discreteNum + 1);
        }
    // 设置参数 ################################################################################################
    inline void setDuration(const double &dur) {
        duration = dur;
        dt = duration / discreteNum;
        updateCoeffTimeMat(); // 更新时间矩阵
    }
    inline void setCoefficentMat(const CoefficientMat &cMat) {
        coeffMat = cMat;
        updateStateVector();  // 更新轨迹段的状态
    }
    inline void setDisceteNum(const int &disNum) {
        discreteNum = disNum;
        dt = duration / discreteNum;
        // updateCoeffTimeMat(); // 更新时间矩阵
    }
    inline void update() {
        updateTimeMat();      // 更新时间矩阵
        updateCoeffTimeMat(); // 更新系数矩阵
        updateStateVector();  // 更新轨迹段的状态
    }
    // 内部实现 ################################################################################################
    inline int getDim() const {
        return 3;
    }

    inline int getOrder() const {
        return 7;
    }

    inline double getDuration() const {
        return duration;
    }

    inline double getDiscretet() const {
        return dt;
    }

    inline int getDiscreteNum() const {
        return discreteNum;
    }
    // c++ 语法 $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
    // inline 中的 [const] 表示常量，返回的数据不能被修改
    // inline 中的 [&] 表示引用，返回的数据是原数据的引用，可以被修改 类似指针一样
    // const {} 中的 [const] 表示常量，表示函数内部不能修改类的成员变量
    inline const CoefficientMat &getCoeffMat() const {
        return coeffMat;
    }
    inline const timeCoefficientMat &getTimeMat() const {
        return timeMat;
    }

    //// maybe not used #################################################################################
    inline const timeCoefficientMat &getTimeMatx() const {
        return xcoefftimeMat;
    }
    inline const timeCoefficientMat &getTimeMaty() const {
        return ycoefftimeMat;
    }
    inline const timeCoefficientMat &getTimeMatq() const {
        return qcoefftimeMat;
    }

    ////Eigen::Matrix3Xd#############################################################################
    inline const Eigen::Matrix3Xd &getPosVector() const {
        return posVector;
    }
    inline const Eigen::Matrix3Xd &getVelVector() const {
        return velVector;
    }
    inline const Eigen::Matrix3Xd &getAccVector() const {
        return accVector;
    }

    ////std::vector<Eigen::Vector3d>##################################################################
    // inline const std::vector<Eigen::Vector3d> &getPosVector() const {
    //     return posVector;
    // }
    // inline const std::vector<Eigen::Vector3d> &getVelVector() const {
    //     return velVector;
    // }
    // inline const std::vector<Eigen::Vector3d> &getAccVector() const {
    //     return accVector;
    // }
    //################################################################################################
    //// get state at time t 单个时刻的计算可能用不着,如何计算一个数据序列的值 这样感觉计算量太大了
    inline Eigen::Vector3d getPos(const double &t) const {
        Eigen::Vector3d pos(0.0, 0.0, 0.0);
        double tn = 1.0;
        for (int i = 0; i <= 7; i++) {
            pos += tn * coeffMat.col(i);
            tn *= t;
        }
        return pos;
    }

    inline Eigen::Vector3d getVel(const double &t) const {
        Eigen::Vector3d vel(0.0, 0.0, 0.0);
        double tn = 1.0;
        int n = 1;
        for (int i = 1; i <= 7; i++) {
            vel += n * tn * coeffMat.col(i);
            tn *= t;
            n++;
        }
        return vel;
    }

    inline Eigen::Vector3d getAcc(const double &t) const {
        Eigen::Vector3d acc(0.0, 0.0, 0.0);
        double tn = 1.0;
        int m = 1;
        int n = 2;
        for (int i = 2; i <= 7; i++) {
            acc += m * n * tn * coeffMat.col(i);
            tn *= t;
            m++;
            n++;
        }
        return acc;
    }

    inline Eigen::Vector3d getJer(const double &t) const {
        Eigen::Vector3d jer(0.0, 0.0, 0.0);
        double tn = 1.0;
        int l = 1;
        int m = 2;
        int n = 3;
        for (int i = 3; i <= 7; i++) {
            jer += l * m * n * tn * coeffMat.col(i);
            tn *= t;
            l++;
            m++;
            n++;
        }
        return jer;
    }
    //################################################################################################
    //// get state at time sequence [ti ~ tf] 单个时刻的计算可能用不着,如何计算一个数据序列的值 这样感觉计算量太大了
    inline void updateStateVector() {
        updatePosVector();
        updateVelVector();
        updateAccVector();
    }

    inline Eigen::Vector3d getMaxVel() const {
        Eigen::Vector3d maxVel(0.0, 0.0, 0.0);
        double maxVelNorm = 0.0,maxOme = 0.0;
        Eigen::Matrix2Xd vel(2, discreteNum + 1);
        Eigen::MatrixXd ome(1, discreteNum + 1);
        vel.row(0) = velVector.row(0);
        vel.row(1) = velVector.row(1);
        ome.row(0) = velVector.row(2);
        for (int idx = 0; idx <= discreteNum; idx++) {
            if (maxVelNorm < vel.col(idx).norm()) {
                maxVelNorm = vel.col(idx).norm();
                maxVel.head(2) = vel.col(idx);
            }
            if (maxOme < std::abs(ome.col(idx)(0))) {
                maxOme = std::abs(ome.col(idx)(0));
                maxVel(2) = ome.col(idx)(0);
            }
        }
        return maxVel;
    }
    inline Eigen::Vector3d getMaxAcc() const {
        Eigen::Vector3d maxAcc(0.0, 0.0, 0.0);
        double maxAccNorm = 0.0,maxAlp = 0.0;
        Eigen::Matrix2Xd acc(2, discreteNum + 1);
        Eigen::MatrixXd alp(1, discreteNum + 1);
        acc.row(0) = accVector.row(0);
        acc.row(1) = accVector.row(1);
        alp.row(0) = accVector.row(2);
        for (int idx = 0; idx <= discreteNum; idx++) {
            if (maxAccNorm < acc.col(idx).norm()) {
                maxAccNorm = acc.col(idx).norm();
                maxAcc.head(2) = acc.col(idx);
            }
            if (maxAlp < std::abs(alp.col(idx)(0))) {
                maxAlp = std::abs(alp.col(idx)(0));
                maxAcc(2) = alp.col(idx)(0);
            }
        }
        return maxAcc;
    }
    ////std::vector<Eigen::Vector3d>##################################################################
    //// @brief std::vector<Eigen::Vector3d> 计算
    // inline void updatePosVector() {
    //     if (!posVector.empty())
    //     posVector.clear();
    //     Eigen::Vector3d pos(0.0, 0.0, 0.0);
    //     for (int idx = 0; idx <= discreteNum; idx++) {
    //     pos = getPos(idx * dt);
    //     posVector.push_back(pos);
    //     }
    // }

    // inline void updateVelVector() {
    //     if (!velVector.empty())
    //     velVector.clear();
    //     Eigen::Vector3d vel(0.0, 0.0, 0.0);
    //     for (int idx = 0; idx <= discreteNum; idx++) {
    //     vel = getVel(idx * dt);
    //     velVector.push_back(vel);
    //     }
    // }

    // inline void updateAccVector() {
    //     if (!accVector.empty())
    //     accVector.clear();
    //     Eigen::Vector3d acc(0.0, 0.0, 0.0);
    //     for (int idx = 0; idx <= discreteNum; idx++) {
    //     acc = getAcc(idx * dt);
    //     accVector.push_back(acc);
    //     }
    // }

    //// get coefficients of normalized polynomial in time [t] 类似计算
    inline void updateCoeffTimeMat() {
        for (int i = 0; i < 4; i++){
            // 将每行的元素进行逐元素乘法，存入xcoefftimeMat
            xcoefftimeMat.row(i) = timeMat.row(i).cwiseProduct(coeffMat.row(0));
            ycoefftimeMat.row(i) = timeMat.row(i).cwiseProduct(coeffMat.row(1));
            qcoefftimeMat.row(i) = timeMat.row(i).cwiseProduct(coeffMat.row(2));
        }
        // xcoefftimeMat = timeMat.array() * coeffMat.row(0).array();
    }
    // inline void updateCoeffTimeMat() {
    //     CoefficientMat CoeffsVector;
    //     double t = 1.0;
    //     for (int i = 7; i >= 0; i--) {
    //         CoeffsVector.col(i) = coeffMat.col(i) * t;
    //         t *= duration;
    //     }
    //     xcoefftimeMat.row(0) = CoeffsVector.row(0);
    //     ycoefftimeMat.row(0) = CoeffsVector.row(1);
    //     qcoefftimeMat.row(0) = CoeffsVector.row(2);
    //     CoeffsVector.setZero();
    //     t = 1.0;    double n = 1.0;
    //     for (int i = 6; i >= 0; i--) {
    //         CoeffsVector.col(i) = n * coeffMat.col(i) * t;
    //         t *= duration;
    //         n++;
    //     }
    //     xcoefftimeMat.row(1) = CoeffsVector.row(0);
    //     ycoefftimeMat.row(1) = CoeffsVector.row(1);
    //     qcoefftimeMat.row(1) = CoeffsVector.row(2);
    //     CoeffsVector.setZero();
    //     t = 1.0;    n = 2.0;    double m = 1.0;
    //     for (int i = 5; i >= 0; i--) {
    //         CoeffsVector.col(i) = n * m * coeffMat.col(i) * t;
    //         t *= duration;
    //         n++;
    //         m++;
    //     }
    //     xcoefftimeMat.row(2) = CoeffsVector.row(0);
    //     ycoefftimeMat.row(2) = CoeffsVector.row(1);
    //     qcoefftimeMat.row(2) = CoeffsVector.row(2);
    //     CoeffsVector.setZero();
    //     t = 1.0;    n = 3.0;    m = 2.0;    double l = 1.0;
    //     for (int i = 4; i >= 0; i--) {
    //         CoeffsVector.col(i) = n * m * l * coeffMat.col(i) * t;
    //         t *= duration;
    //         n++;
    //         m++;
    //         l++;
    //     }
    //     xcoefftimeMat.row(3) = CoeffsVector.row(0);
    //     ycoefftimeMat.row(3) = CoeffsVector.row(1);
    //     qcoefftimeMat.row(3) = CoeffsVector.row(2);
    // }
};

/************************************************************************************************************
 * @description: 
 * @reference: 
 */
class Trajectory {
    private:
    typedef std::vector<Piece> Pieces;
    Pieces pieces;

 public:
    Trajectory() = default;

    Trajectory(const std::vector<double> &durs,const std::vector<int> &disNums,
               const std::vector<CoefficientMat> &cMats) {
        int N = std::min(durs.size(), cMats.size());
        pieces.reserve(N);
        for (int i = 0; i < N; i++) {
            pieces.emplace_back(durs[i], disNums[i], cMats[i]);
        }
    }

    ////get parameters##################################################################################
    inline int getPieceNum() const {
        return pieces.size();
    }

    inline Eigen::VectorXd getDurations() const {
        int N = getPieceNum();
        Eigen::VectorXd durations(N);
        for (int i = 0; i < N; i++) {
            durations(i) = pieces[i].getDuration();
        }
        return durations;
    }

    inline double getTotalDuration() const {
        int N = getPieceNum();
        double totalDuration = 0.0;
        for (int i = 0; i < N; i++) {
            totalDuration += pieces[i].getDuration();
        }
        return totalDuration;
    }

    inline Eigen::VectorXi getDiscreteNums() const {
        int N = getPieceNum();
        Eigen::VectorXi discreteNums(N);
        for (int i = 0; i < N; i++) {
            discreteNums(i) = pieces[i].getDiscreteNum();
        }
        return discreteNums;
    }

    inline int getTotalDiscreteNum() const {
        int N = getPieceNum();
        int totalDiscreteNum = 0;
        for (int i = 0; i < N; i++) {
            totalDiscreteNum += pieces[i].getDiscreteNum();
        }
        return totalDiscreteNum;
    }

    ////set parameters##################################################################################
    inline bool setPieceDuration(const int &idx, const double &dur) {
        if (idx < 0 || idx >= getPieceNum()) {
            return false;
        }
        else {
            pieces[idx].setDuration(dur);
            return true;
        }
    }
    inline bool setPieceCoeffMat(const int &idx, const CoefficientMat &cMat) {
        if (idx < 0 || idx >= getPieceNum()) {
            return false;
        }
        else {
            pieces[idx].setCoefficentMat(cMat);
            return true;
        }
    }
    inline bool setPieceDiscteteNum(const int &idx, const int &disNum) {
        if (idx < 0 || idx >= getPieceNum()) {
            return false;
        }
        else {
            pieces[idx].setDisceteNum(disNum);
            return true;
        }
    }
    inline bool setDuration(const std::vector<double> &durs){
        bool flag = true;
        if(static_cast<int>(durs.size()) == getPieceNum()){
            for(int idx = 0; idx < getPieceNum(); idx++){
                flag = flag && setPieceDuration(idx, durs[idx]);
            }
        }
        else{
            flag = false;
        }
        return flag;
    }
    inline bool setCoeffMat(const std::vector<CoefficientMat> &cMats){
        bool flag = true;
        if(static_cast<int>(cMats.size()) == getPieceNum()){
            for(int idx = 0; idx < getPieceNum(); idx++){
                flag = flag && setPieceCoeffMat(idx, cMats[idx]);
            }
        }
        else{
            flag = false;
        }
        return flag;
    }
    inline bool setDiscteteNum(const std::vector<int> &disNums){
        bool flag = true;
        if(static_cast<int>(disNums.size()) == getPieceNum()){
            for(int idx = 0; idx < getPieceNum(); idx++){
                flag = flag && setPieceDiscteteNum(idx, disNums[idx]);
            }
        }
        else{
            flag = false;
        }
        return flag;
    }

    inline bool resetTraj(const std::vector<double> &durs,const std::vector<int> &disNums,
                        const std::vector<CoefficientMat> &cMats) {
        int N = std::min(durs.size(), cMats.size());
        if (!pieces.empty()) pieces.clear();
            pieces.reserve(N);
        for (int i = 0; i < N; i++) {
            pieces.emplace_back(durs[i], disNums[i], cMats[i]);
        }
        return true;
    }

    inline void updateTraj() {
        int N = getPieceNum();
        for (int i = 0; i < N; i++) {
            pieces[i].update();
        }
    }

    ////################################################################################################
    /// @brief std::vector 的一系列默认操作重写 作用就是可以向操作 std::vector 一样操作 Trajectory
    inline const Piece &operator[](int i) const {
        return pieces[i];
    }
    inline Piece &operator[](int i) {
        return pieces[i];
    }
    inline void clear(void) {
        pieces.clear();
        return;
    }
    inline Pieces::const_iterator begin() const {
        return pieces.begin();
    }
    inline Pieces::const_iterator end() const {
        return pieces.end();
    }
    inline Pieces::iterator begin() {
        return pieces.begin();
    }
    inline Pieces::iterator end() {
        return pieces.end();
    }
    inline void reserve(const int &n) {
        pieces.reserve(n);
        return;
    }
    inline void emplace_back(const Piece &piece) {
        pieces.emplace_back(piece);
        return;
    }
    inline void emplace_back(const double &dur, const int &disNum,
                            const CoefficientMat &cMat) {
        pieces.emplace_back(dur, disNum, cMat);
        return;
    }
    inline void append(const Trajectory &traj) {
        pieces.insert(pieces.end(), traj.begin(), traj.end());
        return;
    }
    ////################################################################################################
    /// @brief get the piece index and the time in the piece at time t and calculate the pos, vel, acc and jer
    /// @param t 
    inline int locatePieceIdx(double &t) const {
        int N = getPieceNum();
        int idx;
        double dur;
        for (idx = 0;
            idx < N &&
            t > (dur = pieces[idx].getDuration());
            idx++) {
            t -= dur;
        }
        if (idx == N) { // t is out of range
            idx--;
            t += pieces[idx].getDuration();
        }
        return idx;
    }
    inline Eigen::Vector3d getPos(double t) const {
        int pieceIdx = locatePieceIdx(t);
        return pieces[pieceIdx].getPos(t);
    }

    inline Eigen::Vector3d getVel(double t) const {
        int pieceIdx = locatePieceIdx(t);
        return pieces[pieceIdx].getVel(t);
    }

    inline Eigen::Vector3d getAcc(double t) const {
        int pieceIdx = locatePieceIdx(t);
        return pieces[pieceIdx].getAcc(t);
    }

    inline Eigen::Vector3d getJer(double t) const {
        int pieceIdx = locatePieceIdx(t);
        return pieces[pieceIdx].getJer(t);
    }

    ////################################################################################################
    /// @brief get the position, velocity and acceleration at the junction
    /// @param juncIdx 
    inline Eigen::Vector3d getJuncPos(int juncIdx) const {
        if (juncIdx != getPieceNum()) {
        return pieces[juncIdx].getCoeffMat().col(7);
        } else {
        return pieces[juncIdx - 1].getPos(pieces[juncIdx - 1].getDuration());
        }
    }
    inline Eigen::Vector3d getJuncVel(int juncIdx) const {
        if (juncIdx != getPieceNum()) {
        return pieces[juncIdx].getCoeffMat().col(6);
        } else {
        return pieces[juncIdx - 1].getVel(pieces[juncIdx - 1].getDuration());
        }
    }
    inline Eigen::Vector3d getJuncAcc(int juncIdx) const {
        if (juncIdx != getPieceNum()) {
        return pieces[juncIdx].getCoeffMat().col(5) * 2.0;
        } else {
        return pieces[juncIdx - 1].getAcc(pieces[juncIdx - 1].getDuration());
        }
    }

    ////################################################################################################
    /// @brief get the MAX position, velocity and acceleration at the trajectory
    /// @return Eigen::Matrix3Xd
    inline Eigen::Vector3d getMaxVel() const {
        int N = getPieceNum();
        Eigen::Vector3d maxVel(0.0, 0.0, 0.0);
        double maxVelNorm = 0.0,maxOme = 0.0;
        for (int idx = 0; idx < N; idx++){
            Eigen::Vector3d vel = pieces[idx].getMaxVel();
            if (maxVelNorm < vel.head(2).norm()) {
                maxVelNorm = vel.head(2).norm();
                maxVel.head(2) = vel.head(2);
            }
            if (maxOme < std::abs(vel(2))) {
                maxOme = std::abs(vel(2));
                maxVel(2) = vel(2);
            }
        }
        return maxVel;
    }
    inline Eigen::Vector3d getMaxAcc() const {
        int N = getPieceNum();
        Eigen::Vector3d maxAcc(0.0, 0.0, 0.0);
        double maxAccNorm = 0.0,maxAlp = 0.0;
        for (int idx = 0; idx < N; idx++){
            Eigen::Vector3d acc = pieces[idx].getMaxAcc();
            if (maxAccNorm < acc.head(2).norm()) {
                maxAccNorm = acc.head(2).norm();
                maxAcc.head(2) = acc.head(2);
            }
            if (maxAlp < std::abs(acc(2))) {
                maxAlp = std::abs(acc(2));
                maxAcc(2) = acc(2);
            }
        }
        return maxAcc;
    }

    ////################################################################################################
    /// @brief get the position, velocity and acceleration at the junction
    /// @return Eigen::Matrix3Xd
    inline Eigen::Matrix3Xd getPositions() const {
        int N = getPieceNum();
        Eigen::Matrix3Xd positions(3, N + 1);
        for (int i = 0; i < N; i++) {
            positions.col(i) = pieces[i].getCoeffMat().col(7);
        }
        positions.col(N) = pieces[N - 1].getPos(pieces[N - 1].getDuration());
        return positions;
    }
    inline Eigen::Matrix3Xd getVelocities() const {
        int N = getPieceNum();
        Eigen::Matrix3Xd velocities(3, N + 1);
        for (int i = 0; i < N; i++) {
            velocities.col(i) = pieces[i].getCoeffMat().col(6);
        }
        velocities.col(N) = pieces[N - 1].getVel(pieces[N - 1].getDuration());
        return velocities;
    }
    inline Eigen::Matrix3Xd getAccelerations() const {
        int N = getPieceNum();
        Eigen::Matrix3Xd accelerations(3, N + 1);
        for (int i = 0; i < N; i++) {
            accelerations.col(i) = pieces[i].getCoeffMat().col(5) * 2.0;
        }
        accelerations.col(N) = pieces[N - 1].getAcc(pieces[N - 1].getDuration());
        return accelerations;
    }
};


#endif