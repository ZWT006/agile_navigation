/*
 * @Author: wentao zhang && zwt190315@163.com
 * @Date: 2023-07-05
 * @LastEditTime: 2023-07-13
 * @Description: Nonlinear Trajectory Optimization
 * @reference: 
 * 
 * Copyright (c) 2023 by wentao zhang, All Rights Reserved. 
 */

#ifndef NONTRAJOPT_HPP_
#define NONTRAJOPT_HPP_

#include <iostream>
#include <vector>
#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>

// osqp-eigen
#include "OsqpEigen/OsqpEigen.h"
#include "nlopt.hpp"
#include "lbfgs_raw.hpp"
#include "osqp.h"

#include <nontrajopt/segment.hpp>
#include <nontrajopt/edfmap.hpp>
#include <nontrajopt/vis_utils.hpp>
#include <nontrajopt/Eigencsv.hpp>

// namespace NonTrajOptSpace
// {

/// @reference: https://github.com/ZJU-FAST-Lab/GCOPTER/blob/main/gcopter/include/gcopter/minco.hpp
// The banded system class is used for solving
// banded linear system Ax=b efficiently.
// A is an N*N band matrix with lower band width lowerBw
// and upper band width upperBw.
// Banded LU factorization has O(N) time complexity.
class BandedSystem {
 public:
  // The size of A, as well as the lower/upper
  // banded width p/q are needed
  inline void create(const int &n, const int &p, const int &q) {
    // In case of re-creating before destroying
    destroy();
    N = n;
    lowerBw = p;
    upperBw = q;
    int actualSize = N * (lowerBw + upperBw + 1);
    ptrData = new double[actualSize];
    std::fill_n(ptrData, actualSize, 0.0);
    return;
  }

  inline void destroy() {
    if (ptrData != nullptr) {
      delete[] ptrData;
      ptrData = nullptr;
    }
    return;
  }

 private:
  int N;
  int lowerBw;
  int upperBw;
  // Compulsory nullptr initialization here
  double *ptrData = nullptr;

 public:
  // Reset the matrix to zero
  inline void reset(void) {
    std::fill_n(ptrData, N * (lowerBw + upperBw + 1), 0.0);
    return;
  }

  // The band matrix is stored as suggested in "Matrix Computation"
  inline const double &operator()(const int &i, const int &j) const {
    return ptrData[(i - j + upperBw) * N + j];
  }

  inline double &operator()(const int &i, const int &j) {
    return ptrData[(i - j + upperBw) * N + j];
  }

  // This function conducts banded LU factorization in place
  // Note that NO PIVOT is applied on the matrix "A" for efficiency!!!
  /// @attention: This function is not numerical stable
  inline void factorizeLU() {
    int iM, jM;
    double cVl;
    for (int k = 0; k <= N - 2; k++) {
      iM = std::min(k + lowerBw, N - 1);
      cVl = operator()(k, k);
      for (int i = k + 1; i <= iM; i++) {
        if (operator()(i, k) != 0.0) {
          operator()(i, k) /= cVl;
        }
      }
      jM = std::min(k + upperBw, N - 1);
      for (int j = k + 1; j <= jM; j++) {
        cVl = operator()(k, j);
        if (cVl != 0.0) {
          for (int i = k + 1; i <= iM; i++) {
            if (operator()(i, k) != 0.0) {
              operator()(i, j) -= operator()(i, k) * cVl;
            }
          }
        }
      }
    }
    return;
  }

  // This function solves Ax=b, then stores x in b
  // The input b is required to be N*m, i.e.,
  // m vectors to be solved.
  inline void solve(Eigen::VectorXd &b) const {
    int iM;
    for (int j = 0; j <= N - 1; j++) {
      iM = std::min(j + lowerBw, N - 1);
      for (int i = j + 1; i <= iM; i++) {
        if (operator()(i, j) != 0.0) {
          b.row(i) -= operator()(i, j) * b.row(j);
        }
      }
    }
    for (int j = N - 1; j >= 0; j--) {
      b.row(j) /= operator()(j, j);
      iM = std::max(0, j - upperBw);
      for (int i = iM; i <= j - 1; i++) {
        if (operator()(i, j) != 0.0) {
          b.row(i) -= operator()(i, j) * b.row(j);
        }
      }
    }
    return;
  }

  // This function solves ATx=b, then stores x in b
  // The input b is required to be N*m, i.e.,
  // m vectors to be solved.
  inline void solveAdj(Eigen::VectorXd &b) const {
    int iM;
    for (int j = 0; j <= N - 1; j++) {
      b.row(j) /= operator()(j, j);
      iM = std::min(j + upperBw, N - 1);
      for (int i = j + 1; i <= iM; i++) {
        if (operator()(j, i) != 0.0) {
          b.row(i) -= operator()(j, i) * b.row(j);
        }
      }
    }
    for (int j = N - 1; j >= 0; j--) {
      iM = std::max(0, j - lowerBw);
      for (int i = iM; i <= j - 1; i++) {
        if (operator()(j, i) != 0.0) {
          b.row(i) -= operator()(j, i) * b.row(j);
        }
      }
    }
    return;
  }
};

// ##############################################################################################################
#define MAT_A_LOWER_BW 30
#define MAT_A_UPPER_BW 10

enum OPT_METHOD {
    LBFGS_RAW,
    NLOPT_LBFGS,
};

struct OptParas;
typedef OptParas* OptParasPtr;

struct OptParas {
    double lambda_smo, lambda_obs, lambda_dyn, lambda_tim, lambda_ova;
    double wq; // weight of state.q coefficients 
    double dist_th; // distance threshold of obstacle
    double discredist; // distance discretization
    double pv_max,pa_max; // max linear velocity, acceleration,
    double wv_max,wa_max; // max angular velocity, acceleration,
    double ORIEN_VEL,VERDIT_VEL; // velocity
    double OVAL_TH; // oval cost threshold
    bool SMO_SWITCH;
    bool OBS_SWITCH;
    bool DYN_SWITCH;
    bool TIM_SWITCH;
    bool OVA_SWITCH;

    bool TIME_OPTIMIZATION;  // time optimization
    bool REDUCE_ORDER;
    //&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
    int    nlopt_max_iteration_num_;  // stopping criteria that can be used
    double nlopt_max_iteration_time_;  // stopping criteria that can be used
    // int    nlopt_max_iteration_num_;  // stopping criteria that can be used
    // double nlopt_max_iteration_time_;  // stopping criteria that can be used
    // int    nlopt_max_iteration_num_;  // stopping criteria that can be used
    // double nlopt_max_iteration_time_;  // stopping criteria that can be used
    OptParas(){};
    ~OptParas(){};
};

/*****************************************************************************************************
//// @brief Nonlinear Trajectory Optimization
//// @reference: https://github.com/ZJU-FAST-Lab/GCOPTER/blob/main/gcopter/include/gcopter/minco.hpp
//// @reference: https://github.com/HKUST-Aerial-Robotics/Fast-Planner/tree/master/fast_planner/bspline_opt
//// @reference: NLopt : https://nlopt.readthedocs.io/en/latest/
*/
class NonTrajOpt
{
    private://##############################################################################################################
    int N;  // number of segments
    int O;  // default order is 7th (8 optimal values)
    int D;  // dimension of the trajectory
    int E;  // number of equality constraints(p v a j)
    int C;  // cost function order v=1 a=2 j=3 s=4; 4th order cost function


    /// @brief: Nonlinear Optimization
    // bound matrix MatABS and upper/lower bound
    int lowerBw;
    int upperBw;
    // &&&&&&&&&&&&&&&&& // full odrer Ax=b solve x
    BandedSystem MatABS;      // Ax=b solve x MatDim*MatDim band matrix
    Eigen::MatrixXd MatA;     // Ax=b solve x MatDim*MatDim full matrix
    Eigen::VectorXd Vecb;   // Ax=b solve b MatDim*1
    Eigen::VectorXd Vecx;   // Ax=b solve x MatDim*1
    // &&&&&&&&&&&&&&&&&
    Eigen::VectorXd Optc;   // 优化变量 coefficients
    Eigen::VectorXd Optt;   // 优化变量 time
    Eigen::VectorXd Optx;   // 优化变量 coefficients + time

    Eigen::VectorXd Vect; // time vector dimensions = N
    Eigen::VectorXd Equb; // 由 waypoints 构建的原始等式约束的右边项 dimensions = EquDim
    // $Ax=b$ and $x$ is the coefficients of the trajectory

    bool TIME_OPTIMIZATION = true;  // time optimization
    // if true Optx = [Optc; Optt] else Optx = Optc
    // Vect = initT; or Vect = exp(Optt)
    bool REDUCE_ORDER = true;       // solve Ax=b to reduce order
    // if true Vecx = Vecb/MatA else Vecx = optc

    // cost, gradient and weight 
    Eigen::VectorXd smogd;
    Eigen::VectorXd obsgd;
    Eigen::VectorXd dyngd;
    Eigen::VectorXd timgd;
    Eigen::VectorXd ovagd;

    double smoCost, obsCost, dynCost, timCost, ovaCost;

    // optimization weight
    double lambda_smo, lambda_obs, lambda_dyn, lambda_tim, lambda_ova;
    double wq; // weight of state.q coefficients 
    double dist_th; // distance threshold of obstacle
    double discredist; // distance discretization
    double pv_max,pa_max,wv_max,wa_max; // max velocity, acceleration,
    double ORIEN_VEL,VERDIT_VEL; // velocity
    double ORIEN_VEL2,VERDIT_VEL2; // velocity^2
    double OVAL_TH; // oval cost threshold

    int    nlopt_max_iteration_num_;  // stopping criteria that can be used
    double nlopt_max_iteration_time_;  // stopping criteria that can be used

    bool SMO_SWITCH;
    bool OBS_SWITCH;
    bool DYN_SWITCH;
    bool TIM_SWITCH;
    bool OVA_SWITCH;

    /// @brief: Quadratic Programming
    /// @reference: argmin 1/2 x^T Q x + f^T x ; s.t. Aeq x = beq
    public:
    Eigen::MatrixXd MatQ;   // QP problem Q
    Eigen::VectorXd QPx;    // QP problem x
    Eigen::MatrixXd MatAeq; // QP problem Aeq
    Eigen::VectorXd Vecbeq; // QP problem beq
    
    private:
    // Temp variables
    // Eigen::MatrixXd RawA;   // Raw A matrix 
    Eigen::VectorXd T1;
    Eigen::VectorXd T2;
    Eigen::VectorXd T3;
    Eigen::VectorXd T4;
    Eigen::VectorXd T5;
    Eigen::VectorXd T6;
    Eigen::VectorXd T7;

    // segments trajectory and edf map
    EDFMap edfmap;
    Trajectory Traj;
    Eigen::Matrix3Xd Waypoints;
    Eigen::Matrix<double, 3, 4> StartStates; 
    Eigen::Matrix<double, 3, 4> EndStates;
    Eigen::VectorXd initT; // initial time
    Eigen::VectorXi discNums; // discretization number of each segment

    Eigen::VectorXd OSQP_optx;
    Eigen::VectorXd Non_optx;

    // %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    // % coffeicents matrixs 的形式为A
    // % q0 q1 q2 q3 q4 q5 q6 q7 q8
    // %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    // % A   coeff = [1,  1*t,  1*t^2,  1*t^3,  1*t^4,  1*t^5,  1*t^6,  1*t^7;
    // %              0,  1,    2*t,    3*t^2,  4*t^3,  5*t^4,  6*t^5,  7*t^6;
    // %              0,  0,    2,      6*t,    12*t^2, 20*t^3, 30*t^4, 42*t^5;
    // %              0,  0,    0,      6,      24*t,   60*t^2, 120*t^3,210*t^4];
    // %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    // % factorial(coll-1)/factorial(coll-row) = n_order的k阶导的系数
    // % t^(coll-row) 该项阶次
    // %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    inline int factorial(int n) ;
    inline Eigen::MatrixXd getCoeffCons(const double t, const int order, const int equdim);
    inline Eigen::MatrixXd getCoeffCons(const double t);
    inline Eigen::VectorXd getTimeVec(const double t, const int row );
    inline double getSegPolyCost(const Eigen::VectorXd &coef,int idx);
    inline void getSegPolyGradc(Eigen::VectorXd &coef,int idx);
    inline double getSegPolyGradt(const Eigen::VectorXd &coef,int idx);

    // void calcuSmoCost();     // calculate smoothness cost
    // void calcuObsCost();     // calculate obstacle cost
    // void calcuDynCost();     // calculate dynamic cost
    // void calcuTimCost();     // calculate time cost
    // void calcuOvaCost();     // calculate overall cost double 

    void calcuSmoCost(double &Cost,Eigen::VectorXd gradc, Eigen::VectorXd gradt);       // calculate smoothness cost
    void calcuObsCost(double &Cost,Eigen::VectorXd gradc, Eigen::VectorXd gradt);       // calculate obstacle cost
    void calcuDynCost(double &Cost,Eigen::VectorXd gradc, Eigen::VectorXd gradt);       // calculate dynamic cost
    void calcuTimCost(double &Cost,Eigen::VectorXd gradc, Eigen::VectorXd gradt);       // calculate time cost
    void calcuOvaCost(double &Cost,Eigen::VectorXd gradc, Eigen::VectorXd gradt);       // calculate overall cost

    /* data */
    public://##############################################################################################################
    int OptDof; // Degree of freedom of optimization
    int EquDim; // dimension of equality constraints
    int MatDim; // dimension of matrix A
    int OptNum; // number of optimization variables
    OPT_METHOD optMethod = LBFGS_RAW;

    // for outside use gdopt = [gdcoe; gdtau] 其中 gdcoe 是部分行的梯度
    // Eigen::VectorXi isOpt; // isOpt[i] = true 表示第i个系数是优化变量
    Eigen::VectorXd gdcoe; // gradient of coefficients full size
    Eigen::VectorXd gdtau; // gradient of time
    Eigen::VectorXd gdopt; // gradient of optimization variables

    std::vector<int> isOpt; // isOpt[i]  表示第n个变量是优化变量
    std::vector<int> noOpt; // noOpt[i]  表示第n个系数不是优化变量
    std::vector<bool> optFlag; // optFlag[i]  表示第n个系数是否是优化变量

    NonTrajOpt(/* args */) = default;
    ~NonTrajOpt() { MatABS.destroy();};

    // fixed parameters for every optimization 
    inline void initParameter(const OptParas &paras);

    // every time optimization reset parameters
    inline void reset(const int &pieceNum);
    // init waypoints and start/end states  for polynomial trajectory optimization
    inline void initWaypoints(const Eigen::Matrix3Xd &_waypoints, const Eigen::VectorXd &_initT, 
                              const Eigen::Matrix<double, 3, 4> &_startStates, const Eigen::Matrix<double, 3, 4> &_endStates);
    inline void getReduceOrder(Eigen::VectorXd &_vec); // reduce order vector

    // key functions &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
    inline void updateTime() ;
    inline void updateOptVars(const Eigen::VectorXd &_optvar);  // update optimization variables
    // for Nonlinear Optimization
    inline void updateOptAxb();                              // update trajectory Matrix by Optx
    // for QP problem
    inline void updateAeqbeq();    // update MatA, Vecb by time initT  
    inline void updateMatQ();      // update MatQ by time initT
    // for Trajectory Traj
    inline void updateTraj();      // update Traj by Vecx and Vect
    
    void calcuOvaConstriants();                       // calculate overall constraints

    static double NLoptobjection(const std::vector<double>& optx, std::vector<double>& grad,void* func_data);
    double LBFGSobjection(void* ptrObj,const double* optx,double* grad,const int n);

    bool OSQPSolve();
    bool NLoptSolve();
    bool LBFGSSolve();
};
// ##############################################################################################################
// private inline auxiliary functions############################################################################
// ##############################################################################################################

inline int NonTrajOpt::factorial(int n) {
        return (n == 1 || n == 0) ? 1 : factorial(n - 1) * n;
    }

inline Eigen::MatrixXd NonTrajOpt::getCoeffCons(const double t, const int order, const int equdim) {
    Eigen::MatrixXd coeff = Eigen::MatrixXd::Zero(equdim, order);
    for (int row = 0;row < equdim;row++) {
        for (int col = row;col < order;col++) {
            coeff(row, col) = factorial(col)/factorial(col-row)*std::pow(t,col-row);
        }
    }
    return coeff;
}
inline Eigen::MatrixXd NonTrajOpt::getCoeffCons(const double t) {
    Eigen::MatrixXd coeff = Eigen::MatrixXd::Zero(E, O);
    for (int row = 0; row < E; row++) {
        for (int col = row; col < O; col++) {
            coeff(row, col) = factorial(col)/factorial(col-row)*std::pow(t,col-row);
        }
    }
    return coeff;
}

// ##############################################################################################################
// External calls key functions #################################################################################
// ##############################################################################################################
inline void NonTrajOpt::initParameter(const OptParas &paras) {
    lambda_smo = paras.lambda_smo; 
    lambda_obs = paras.lambda_obs; 
    lambda_dyn = paras.lambda_dyn; 
    lambda_tim = paras.lambda_tim;
    lambda_ova = paras.lambda_ova;
    wq         = paras.wq; // weight of state.q coefficients 
    dist_th    = paras.dist_th; // distance threshold of obstacle
    pv_max     = paras.pv_max;
    pa_max     = paras.pa_max;
    wv_max     = paras.wv_max;
    wa_max     = paras.wa_max; // max velocity, acceleration,
    discredist = paras.discredist; // distance discretization
    ORIEN_VEL  = paras.ORIEN_VEL;
    VERDIT_VEL = paras.VERDIT_VEL; // velocity
    ORIEN_VEL2 = ORIEN_VEL * ORIEN_VEL; // velocity orientaion
    VERDIT_VEL2 = VERDIT_VEL * VERDIT_VEL; // velocity vertical

    TIME_OPTIMIZATION   = paras.TIME_OPTIMIZATION;  // time optimization
    REDUCE_ORDER        = paras.REDUCE_ORDER;

    //&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
    // Optimization Solver Parameters
    nlopt_max_iteration_num_ = paras.nlopt_max_iteration_num_;
    nlopt_max_iteration_time_ = paras.nlopt_max_iteration_time_;


    // always fixed parameters
    lowerBw = MAT_A_LOWER_BW;
    upperBw = MAT_A_UPPER_BW;
    O = 8; // 7th = 8 coefficients
    D = 3; // [x,y,q]
    E = 4; // [p,v,a,j]
    C = 4; // cost function order
}

inline void NonTrajOpt::initWaypoints(const Eigen::Matrix3Xd &_waypoints, const Eigen::VectorXd &_initT, 
        const Eigen::Matrix<double, 3, 4> &_startStates, const Eigen::Matrix<double, 3, 4> &_endStates) {
    Waypoints = _waypoints;
    StartStates = _startStates;
    EndStates = _endStates;
    initT = _initT;
    Vect = initT;
    double dist = 0.0;
    for (int i = 0; i < N; i++) {
        Eigen::Vector2d xypos = Waypoints.row(i).head(2);
        dist = xypos.norm();
        discNums(i) = std::ceil(dist/discredist);
    }
    updateTime();
}
// ##############################################################################################################
// public auxiliary functions for parameters update #############################################################
// ##############################################################################################################
inline void NonTrajOpt::getReduceOrder(Eigen::VectorXd &_vec) {
    Eigen::VectorXd re_vec = Eigen::VectorXd::Zero(OptDof);
    // update OptDof equality constraints
    int dof_num = 0;
    for (int idx = 0; idx < MatDim && dof_num < OptDof; idx++) {
        if (optFlag[idx]) { // true is optimization variable
            re_vec(dof_num) = _vec(idx);
            dof_num++;
        }
    }
    // 这应该就是正常的赋值操作吧?
    _vec = re_vec;
}
inline void NonTrajOpt::updateTime() {
    // cwiseProduct() 函数是逐元素相乘
    T1 = Vect;
    T2 = T1.cwiseProduct(T1);
    T3 = T2.cwiseProduct(T1);
    T4 = T2.cwiseProduct(T2);
    T5 = T3.cwiseProduct(T2);
    T6 = T3.cwiseProduct(T3);
    T7 = T4.cwiseProduct(T3);
}

inline void NonTrajOpt::updateOptVars(const Eigen::VectorXd &_optvar) {
    Optx = _optvar;
    if (TIME_OPTIMIZATION) { // 优化时间所以每次都要更新
        Optt = Optx.tail(N);
        Vect = Optt.array().exp();
        updateTime();
    }
    else {  // 时间固定所以不用更新
        Vect = initT;
    }
    if (REDUCE_ORDER)
        Optc = Optx.head(OptDof);
    else
        Optc = Optx.head(MatDim);
}

// ##############################################################################################################
// optimization key functions ###################################################################################
// ##############################################################################################################

inline void NonTrajOpt::reset(const int &pieceNum) {
    N = pieceNum;
    MatDim = N * O * D;
    EquDim = 2*E*D + E*(N-1)*D + (N-1)*D;
    // opt 问题的自由度 整个自由度 减去 等式约束 (waypoints连续性约束+waypoints固定点约束)
    OptDof = MatDim - EquDim;
    MatABS.create(MatDim, lowerBw, upperBw);
    MatA.resize(MatDim, MatDim);
    Vecb.resize(MatDim);
    Vecx.resize(MatDim);

    Optt.resize(N);
    int OptxDof = 0;
    if (REDUCE_ORDER)   OptxDof += OptDof;
    else    OptxDof += MatDim;
    Optc.resize(OptxDof);
    
    if (TIME_OPTIMIZATION)  OptxDof += N;
    Optx.resize(OptxDof);
    
    OptNum = OptxDof;

    Vect.resize(N);
    Equb.resize(EquDim);

    if (Traj.getPieceNum()!=0)
        Traj.clear();
    Traj.reserve(N);
    Waypoints.resize(N + 1, D);
    StartStates.setZero();
    EndStates.setZero();
    initT.resize(N);
    discNums.resize(N);
    // QP problem resize &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
    MatQ.resize(MatDim, MatDim);
    QPx.resize(MatDim);
    MatAeq.resize(EquDim, MatDim);
    Vecbeq.resize(EquDim);

    isOpt.resize(MatDim);
    gdcoe.resize(MatDim); // gradient of coefficients full size
    gdtau.resize(N); // gradient of time
    gdopt.resize(OptxDof); // gradient of optimization variables

    OSQP_optx.resize(MatDim);
    Non_optx.resize(OptNum);

    if (!isOpt.empty()) isOpt.clear();
    if (!noOpt.empty()) noOpt.clear();
    if (!optFlag.empty()) optFlag.clear();
    optFlag.resize(MatDim,false);
    ////TODO: 可以在 reset() 的时候就初始化 isOpt 和 noOpt 两个向量 后边少一点点计算量
    int dof_num = 0; //NOTE: 感觉这里的 第二和第三个循环的顺序不影响问题求解 但是不太好
    for (int id_seg = 0; id_seg < N  && dof_num < OptDof; id_seg++) { // N segments
        for (int id_dim = 0; id_dim < D && dof_num < OptDof; id_dim++) { // 3 dimensions
            for (int id_ord = E +1 ; id_ord < O && dof_num < OptDof; id_ord++) { // 5~7 orders
                int id_num = id_seg*O*D + id_dim*O + id_ord;
                optFlag[id_num] = true;
                dof_num++;
            }
        }
    }
    
}

void NonTrajOpt::updateOptAxb() {
    Eigen::MatrixXd TempAeq = Eigen::MatrixXd::Zero(EquDim, MatDim);
    Eigen::VectorXd Tempbeq = Eigen::VectorXd::Zero(EquDim);
    //// update MatA by StartState &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
    Eigen::MatrixXd Aeq_start = Eigen::MatrixXd::Zero(E*D, MatDim);
    Eigen::VectorXd beq_start = Eigen::VectorXd::Zero(E*D);
    for(int i = 0; i < D; i++) {
        Aeq_start.block(i*E, i*O, E, O) = getCoeffCons(0.0);
        beq_start.segment(i*E, E) = StartStates.row(i).transpose();
    }
    // update MatA by Waypoints &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
    Eigen::MatrixXd Aeq_wp = Eigen::MatrixXd::Zero(E*(N-1)*D + (N-1)*D, MatDim);
    Eigen::VectorXd beq_wp = Eigen::VectorXd::Zero(E*(N-1)*D + (N-1)*D);
    Eigen::MatrixXd Matn = -1*getCoeffCons(0.0);
    for (int k = 0; k < N-1; k++) {
        double ti = Vect(k);
        Eigen::MatrixXd coeff = getCoeffCons(ti,O,1);
        Eigen::MatrixXd Matp = getCoeffCons(ti);
        for(int i = 0; i < D; i++) {
            int KDi = k*D+i;
            // waypoints pose constraints
            beq_wp((E+1)*KDi,1) = Waypoints(i,k+1);
            // 1:t:t^2:t^3:t^4…… * [p0 p1 p2 p3……]T
            Aeq_wp.block((E+1)*KDi, KDi*O, 1, O) = coeff;
            Aeq_wp.block((E+1)*KDi + 1, KDi*O, 4, O) = Matp;
            Aeq_wp.block((E+1)*KDi + 1, (KDi+D)*O, 4, O) = Matn;
        }
    }
    //// update MatA by EndState &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
    Eigen::MatrixXd Aeq_end = Eigen::MatrixXd::Zero(E*D, MatDim);
    Eigen::VectorXd beq_end = Eigen::VectorXd::Zero(E*D);
    double te = Vect.tail(1)(0); // last element
    for(int i = 0; i < D; i++) {
        Aeq_end.block(i*E, (N-1)*D*O+i*O, E, O) = getCoeffCons(te);
        beq_end.segment(i*E, E) = EndStates.row(i).transpose();
    }

    TempAeq.block(0, 0, Aeq_start.rows(), MatDim) = Aeq_start;
    TempAeq.block(Aeq_start.rows(), 0, Aeq_wp.rows(), MatDim) = Aeq_wp;
    TempAeq.block(Aeq_start.rows()+Aeq_wp.rows(), 0, Aeq_end.rows(), MatDim) = Aeq_end;
    Tempbeq.segment(0, beq_start.rows()) = beq_start;
    Tempbeq.segment(beq_start.rows(), beq_wp.rows()) = beq_wp;
    Tempbeq.segment(beq_start.rows()+beq_wp.rows(), beq_end.rows()) = beq_end;

    EigenCSV eigenCSV;
    //// test TempAeq and Tempbeq = Ok
    // std::cout << "Write TempAeqRAW.csv" << std::endl;
    // eigenCSV.WriteMatrix(TempAeq,"/home/zwt/Documents/TempAeqRAW.csv");
    // eigenCSV.WriteVector(Tempbeq,"/home/zwt/Documents/TempbeqRAW.csv");

    Eigen::MatrixXd TempAeqRE = Eigen::MatrixXd::Zero(MatDim, MatDim);
    Eigen::VectorXd TempbeqRE = Eigen::VectorXd::Zero(MatDim);

    // int equ_num = 0;
    // for (int id_seg = 0; id_seg < N && equ_num < EquDim; id_seg++) { // N segments
    //     for (int id_dim = 0; id_dim < D && equ_num < EquDim; id_dim++) { // 3 dimensions
    //         for (int id_ord = 0 ; id_ord < E + 1 && equ_num < EquDim; id_ord++) { // 0~4 orders
    //             int id_num = id_seg*O*D + id_dim*O + id_ord;
    //             TempAeqRE.row(id_num) = TempAeq.row(equ_num);
    //             TempbeqRE(id_num) = Tempbeq(equ_num);
    //             // std::cout << "equ row : " << id_num << " " << TempAeq.row(equ_num) << std::endl;
    //             std::cout << "equ row : " << id_num  << std::endl;
    //             equ_num++;
    //         }
    //     }
    // }
    // std::cout << "equ num : " << equ_num << std::endl;

    //// update OptDof equality constraints
    int dof_num = 0; //NOTE: 感觉这里的 第二和第三个循环的顺序不影响问题求解 但是不太好
    for (int id_seg = 0; id_seg < N  && dof_num < OptDof; id_seg++) { // N segments
        for (int id_dim = 0; id_dim < D && dof_num < OptDof; id_dim++) { // 3 dimensions
            for (int id_ord = E +1 ; id_ord < O && dof_num < OptDof; id_ord++) { // 5~7 orders
                int id_num = id_seg*O*D + id_dim*O + id_ord;
                Eigen::VectorXd tempRow = Eigen::VectorXd::Zero(MatDim);
                tempRow(id_num) = 1.0;
                TempAeqRE.row(id_num) = tempRow.transpose();
                // std::cout << "dof row : " << id_num << " " << tempRow.transpose() << std::endl;
                TempbeqRE(id_num) = Optx(dof_num);
                dof_num++;
            }
        }
    }
    std::cout << "dof num : " << dof_num << std::endl;

    int equ_num = 0;
    for (int idx = 0; idx < MatDim && equ_num < EquDim; idx++) {
        if (!optFlag[idx]) {
            TempAeqRE.row(idx) = TempAeq.row(equ_num);
            TempbeqRE(idx) = Tempbeq(equ_num);
            equ_num++;
        }
    }
    std::cout << "equ num : " << equ_num << std::endl;
    

    std::cout << "Write TempAeqRE.csv" << std::endl;
    eigenCSV.WriteMatrix(TempAeqRE,"/home/zwt/Documents/TempAeqRE.csv");
    eigenCSV.WriteVector(TempbeqRE,"/home/zwt/Documents/TempbeqRE.csv");

    // Debug &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
    std::cout << "TempAeq sizes: " << TempAeq.rows() << " " << TempAeq.cols() << std::endl;
    std::cout << "TempAeq rank: " << TempAeq.fullPivLu().rank() << std::endl;
    std::cout << "TempAeqRE sizes: " << TempAeqRE.rows() << " " << TempAeq.cols() << std::endl;
    std::cout << "TempAeqRE rank: " << TempAeqRE.fullPivLu().rank() << std::endl;
    // 这里可以使用 fullPivLu() 类来求解，也可以使用 colPivHouseholderQr() 类来求解
    // 但是 GCOPTER 中使用的就是自己写的 BandedSystem 类来求解  可能是因为稀疏矩阵求解效率的原因吧 可与对比对比
    // TODO: 求解线性方程组 Ax=b &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
    MatA = TempAeqRE;
    Vecx = TempAeqRE.fullPivLu().solve(TempbeqRE);
    std::cout << "Vecx sizes: " << Vecx.rows() << " " << Vecx.cols() << std::endl;
    std::cout << "Vecx: " << Vecx.transpose() << std::endl;
    // 将 TempAeq 和 Tempbeq 赋值给 MatABS 和 Vecb
    Vecb = TempbeqRE;

    // for (int i = 0; i < MatDim; i++) {
    //     for (int j = 0; j < MatDim; j++) {
    //         if ((i - j) <= lowerBw && (j - i) <= upperBw) {
    //             MatABS(i, j) = TempAeq(i,j);  // 赋值为 1.0，你可以根据需要修改
    //         }
    //     }
    // }
    // std::cout << "BandedSystem MatABS GG?" << std::endl;
    // MatABS.factorizeLU();
    // MatABS.solve(TempbeqRE);
    // std::cout << "BandedSystem MatABS GG!" << std::endl;
    // std::cout << "Solve : " << TempbeqRE.transpose() << std::endl;

}

inline void NonTrajOpt::updateTraj() {
    std::vector<CoefficientMat> coeffMats;
    for (int i = 0; i < N; i++){
        CoefficientMat coeffMat;
        coeffMat.block(0,0,0,O) = Vecx.segment(i*O*D, O);
        coeffMat.block(1,0,1,O) = Vecx.segment(i*O*D+O, O);
        coeffMat.block(2,0,2,O) = Vecx.segment(i*O*D+2*O, O);
        coeffMats.push_back(coeffMat);
    }
    std::vector<double> times(initT.data(), initT.data() + initT.size());
    std::vector<int> discs(discNums.data(), discNums.data() + discNums.size());
    Traj.resetTraj(times, discs, coeffMats);
    Traj.updateTraj(); // update coefficients, time and state vectors of trajectory
}

// ##############################################################################################################
/// @description: calculate Q and Aeq/beq matrix by time initT to solve QP problem 
void NonTrajOpt::updateMatQ() {
    // Calculate Q matrix
    MatQ.setZero();
    for (int n = 0; n < N; n++) {
        Eigen::MatrixXd Qn = Eigen::MatrixXd::Zero(O, O);
        double ts = initT(n);
        for (int i = C; i < O;i++){
            for (int j = C; j < O;j++){
                Qn(i,j) = factorial(i)/factorial(i-C)*factorial(j)/factorial(j-C)*pow(ts, i+j-2*C+1)/(i+j-2*C+1);
            }
        }
        int nOD = n*O*D;
        MatQ.block(nOD, nOD, O, O) = Qn;
        MatQ.block(nOD+O, nOD+O, O, O) = Qn;
        Qn.diagonal() *=  wq;
        MatQ.block(nOD+2*O, nOD+2*O, O, O) = Qn;
    }
}

void NonTrajOpt::updateAeqbeq() {
    // StartState Constraints &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
    Eigen::MatrixXd Aeq_start = Eigen::MatrixXd::Zero(E*D, MatDim);
    Eigen::VectorXd beq_start = Eigen::VectorXd::Zero(E*D);
    for(int i = 0; i < D; i++) {
        Aeq_start.block(i*E, i*O, E, O) = getCoeffCons(0.0);
        beq_start.segment(i*E, E) = StartStates.row(i).transpose();
    }
    // Waypoints and Equality Constraints &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
    Eigen::MatrixXd Aeq_wp = Eigen::MatrixXd::Zero(E*(N-1)*D + (N-1)*D, MatDim);
    Eigen::VectorXd beq_wp = Eigen::VectorXd::Zero(E*(N-1)*D + (N-1)*D);
    Eigen::MatrixXd Matn = -1*getCoeffCons(0.0);
    for (int k = 0; k < N-1; k++) {
        double ti = initT(k);
        Eigen::MatrixXd coeff = getCoeffCons(ti,O,1);
        Eigen::MatrixXd Matp = getCoeffCons(ti);
        for(int i = 0; i < D; i++) {
            int KDi = k*D+i;
            // waypoints pose constraints
            beq_wp((E+1)*KDi,1) = Waypoints(i,k+1);
            // 1:t:t^2:t^3:t^4…… * [p0 p1 p2 p3……]T
            Aeq_wp.block((E+1)*KDi, KDi*O, 1, O) = coeff;
            Aeq_wp.block((E+1)*KDi + 1, KDi*O, E, O) = Matp;
            Aeq_wp.block((E+1)*KDi + 1, (KDi+D)*O, E, O) = Matn;
        }
    }
    // EndState Constraints &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
    Eigen::MatrixXd Aeq_end = Eigen::MatrixXd::Zero(E*D, MatDim);
    Eigen::VectorXd beq_end = Eigen::VectorXd::Zero(E*D);
    double te = initT.tail(1)(0); // last element
    for(int i = 0; i < D; i++) {
        Aeq_end.block(i*E, (N-1)*D*O+i*O, E, O) = getCoeffCons(te);
        beq_end.segment(i*E, E) = EndStates.row(i).transpose();
    }
    // Constraints Matrix &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
    // std::cout << "MatAeq sizes" << MatAeq.rows() << " " << MatAeq.cols() << std::endl;
    // std::cout << "Aeq row" << Aeq_start.rows()+Aeq_wp.rows()+Aeq_end.rows() << std::endl;
    // std::cout << "beq row" << beq_start.rows()+beq_wp.rows()+beq_end.rows() << std::endl;
    // $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
    MatAeq.block(0, 0, Aeq_start.rows(), MatDim) = Aeq_start;
    MatAeq.block(Aeq_start.rows(), 0, Aeq_wp.rows(), MatDim) = Aeq_wp;
    MatAeq.block(Aeq_start.rows()+Aeq_wp.rows(), 0, Aeq_end.rows(), MatDim) = Aeq_end;
    Vecbeq.segment(0, beq_start.rows()) = beq_start;
    Vecbeq.segment(beq_start.rows(), beq_wp.rows()) = beq_wp;
    Vecbeq.segment(beq_start.rows()+beq_wp.rows(), beq_end.rows()) = beq_end;
}



// ##############################################################################################################
// Objective Functions 
// ##############################################################################################################
double NonTrajOpt::NLoptobjection(const std::vector<double>& x, std::vector<double>& grad,
                                  void* func_data) {
    NonTrajOpt* OPTptr = reinterpret_cast<NonTrajOpt*>(func_data);
    double Cost;
    grad.resize(OPTptr->OptNum);
    std::fill(grad.begin(), grad.end(), 0.0);
    Eigen::VectorXd _optx = Eigen::Map<const Eigen::VectorXd>(x.data(), x.size());
    Eigen::VectorXd _grad = Eigen::Map<Eigen::VectorXd>(grad.data(), grad.size());
    // 更新优化变量
    OPTptr->updateOptVars(_optx);
    // 更新优化变量对应的矩阵 并求解 Ax=b
    OPTptr->updateOptAxb();
    // 将求解后的系数放入 Traj 中
    OPTptr->updateTraj();
    // 计算 cost

    if (OPTptr->SMO_SWITCH) {
        double smoCost = 0;
        Eigen::VectorXd gradc = Eigen::VectorXd::Zero(OPTptr->OptDof);
        Eigen::VectorXd gradt = Eigen::VectorXd::Zero(OPTptr->N);
        OPTptr->calcuSmoCost(smoCost,gradc,gradt);
        Cost += smoCost * OPTptr->lambda_smo;
        OPTptr->getReduceOrder(gradc);
        gradc *= OPTptr->lambda_smo;
        gradt *= OPTptr->lambda_smo;
        _grad.head(OPTptr->OptDof) += gradc;
        _grad.tail(OPTptr->N) += gradt;
    }
    if (OPTptr->OBS_SWITCH) {
        double obsCost = 0;
        Eigen::VectorXd gradc = Eigen::VectorXd::Zero(OPTptr->OptDof);
        Eigen::VectorXd gradt = Eigen::VectorXd::Zero(OPTptr->N);
        OPTptr->calcuObsCost(obsCost,gradc,gradt);
        Cost += obsCost * OPTptr->lambda_obs;
        OPTptr->getReduceOrder(gradc);
        gradc *= OPTptr->lambda_obs;
        gradt *= OPTptr->lambda_obs;
        _grad.head(OPTptr->OptDof) += gradc;
        _grad.tail(OPTptr->N) += gradt;
    }
    if (OPTptr->DYN_SWITCH) {
        double dynCost = 0;
        Eigen::VectorXd gradc = Eigen::VectorXd::Zero(OPTptr->OptDof);
        Eigen::VectorXd gradt = Eigen::VectorXd::Zero(OPTptr->N);
        OPTptr->calcuDynCost(dynCost,gradc,gradt);
        Cost += dynCost * OPTptr->lambda_dyn;
        OPTptr->getReduceOrder(gradc);
        gradc *= OPTptr->lambda_dyn;
        gradt *= OPTptr->lambda_dyn;
        _grad.head(OPTptr->OptDof) += gradc;
        _grad.tail(OPTptr->N) += gradt;
    }
    if (OPTptr->TIM_SWITCH) {
        double timCost = 0;
        Eigen::VectorXd gradc = Eigen::VectorXd::Zero(OPTptr->OptDof);
        Eigen::VectorXd gradt = Eigen::VectorXd::Zero(OPTptr->N);
        OPTptr->calcuTimCost(timCost,gradc,gradt);
        Cost += timCost * OPTptr->lambda_tim;
        OPTptr->getReduceOrder(gradc);
        gradc *= OPTptr->lambda_tim;
        gradt *= OPTptr->lambda_tim;
        _grad.head(OPTptr->OptDof) += gradc;
        _grad.tail(OPTptr->N) += gradt;
    }
    if (OPTptr->OVA_SWITCH) {
        double Ovacost = 0;
        Eigen::VectorXd gradc = Eigen::VectorXd::Zero(OPTptr->OptDof);
        Eigen::VectorXd gradt = Eigen::VectorXd::Zero(OPTptr->N);
        OPTptr->calcuOvaCost(Ovacost,gradc,gradt);
        Cost += Ovacost * OPTptr->lambda_ova;
        OPTptr->getReduceOrder(gradc);
        gradc *= OPTptr->lambda_ova;
        gradt *= OPTptr->lambda_ova;
        _grad.head(OPTptr->OptDof) += gradc;
        _grad.tail(OPTptr->N) += gradt;
    }
    // 将 Eigen::VectorXd _grad 赋值给 std::vector<double> grad
    grad.assign(_grad.data(), _grad.data() + _grad.size());
    return Cost;
}

double NonTrajOpt::LBFGSobjection(void* ptrObj,const double* optx,double* grad,const int n) {
    // NonTrajOpt& obj = *(NonTrajOpt*)ptrObj;
    NonTrajOpt* OPTptr = reinterpret_cast<NonTrajOpt*>(ptrObj);
    double Cost;
    std::fill(grad, grad + n, 0.0);
    Eigen::VectorXd _optx = Eigen::Map<const Eigen::VectorXd>(optx, n);
    Eigen::VectorXd _grad = Eigen::Map<Eigen::VectorXd>(grad, n);
    // 更新优化变量
    OPTptr->updateOptVars(_optx);
    // 更新优化变量对应的矩阵 并求解 Ax=b
    OPTptr->updateOptAxb();
    // 将求解后的系数放入 Traj 中
    OPTptr->updateTraj();
    // 计算 cost

    if (OPTptr->SMO_SWITCH) {
        double smoCost = 0;
        Eigen::VectorXd gradc = Eigen::VectorXd::Zero(OPTptr->OptDof);
        Eigen::VectorXd gradt = Eigen::VectorXd::Zero(OPTptr->N);
        OPTptr->calcuSmoCost(smoCost,gradc,gradt);
        Cost += smoCost * OPTptr->lambda_smo;
        OPTptr->getReduceOrder(gradc);
        gradc *= OPTptr->lambda_smo;
        gradt *= OPTptr->lambda_smo;
        _grad.head(OPTptr->OptDof) += gradc;
        _grad.tail(OPTptr->N) += gradt;
    }
    if (OPTptr->OBS_SWITCH) {
        double obsCost = 0;
        Eigen::VectorXd gradc = Eigen::VectorXd::Zero(OPTptr->OptDof);
        Eigen::VectorXd gradt = Eigen::VectorXd::Zero(OPTptr->N);
        OPTptr->calcuObsCost(obsCost,gradc,gradt);
        Cost += obsCost * OPTptr->lambda_obs;
        OPTptr->getReduceOrder(gradc);
        gradc *= OPTptr->lambda_obs;
        gradt *= OPTptr->lambda_obs;
        _grad.head(OPTptr->OptDof) += gradc;
        _grad.tail(OPTptr->N) += gradt;
    }
    if (OPTptr->DYN_SWITCH) {
        double dynCost = 0;
        Eigen::VectorXd gradc = Eigen::VectorXd::Zero(OPTptr->OptDof);
        Eigen::VectorXd gradt = Eigen::VectorXd::Zero(OPTptr->N);
        OPTptr->calcuDynCost(dynCost,gradc,gradt);
        Cost += dynCost * OPTptr->lambda_dyn;
        OPTptr->getReduceOrder(gradc);
        gradc *= OPTptr->lambda_dyn;
        gradt *= OPTptr->lambda_dyn;
        _grad.head(OPTptr->OptDof) += gradc;
        _grad.tail(OPTptr->N) += gradt;
    }
    if (OPTptr->TIM_SWITCH) {
        double timCost = 0;
        Eigen::VectorXd gradc = Eigen::VectorXd::Zero(OPTptr->OptDof);
        Eigen::VectorXd gradt = Eigen::VectorXd::Zero(OPTptr->N);
        OPTptr->calcuTimCost(timCost,gradc,gradt);
        Cost += timCost * OPTptr->lambda_tim;
        OPTptr->getReduceOrder(gradc);
        gradc *= OPTptr->lambda_tim;
        gradt *= OPTptr->lambda_tim;
        _grad.head(OPTptr->OptDof) += gradc;
        _grad.tail(OPTptr->N) += gradt;
    }
    if (OPTptr->OVA_SWITCH) {
        double Ovacost = 0;
        Eigen::VectorXd gradc = Eigen::VectorXd::Zero(OPTptr->OptDof);
        Eigen::VectorXd gradt = Eigen::VectorXd::Zero(OPTptr->N);
        OPTptr->calcuOvaCost(Ovacost,gradc,gradt);
        Cost += Ovacost * OPTptr->lambda_ova;
        OPTptr->getReduceOrder(gradc);
        gradc *= OPTptr->lambda_ova;
        gradt *= OPTptr->lambda_ova;
        _grad.head(OPTptr->OptDof) += gradc;
        _grad.tail(OPTptr->N) += gradt;
    }
    // 更新优化变量 将 Eigen::VectorXd _grad 赋值给 double* grad
    std::memcpy(grad, _grad.data(), N * sizeof(double));
    return Cost;
}
// ##############################################################################################################
// Optimization Solvers #########################################################################################
// ##############################################################################################################
// OSQP Function
bool NonTrajOpt::OSQPSolve() {
    bool falg = false;

    // updateAeqbeq();    // update MatA, Vecb by time initT  
    // updateMatQ();      // update MatQ by time initT

    Eigen::SparseMatrix<double> Hessian_sparse = MatQ.sparseView();

    Eigen::SparseMatrix<double> MatAeq_sparse = MatAeq.sparseView();

    Eigen::VectorXd gradient = Eigen::VectorXd::Zero(MatDim);
    Eigen::VectorXd lowerBound = Vecbeq.sparseView();
    Eigen::VectorXd upperBound = Vecbeq.sparseView();
    Eigen::VectorXd QPoptx = Eigen::VectorXd::Zero(MatDim);

    lowerBound -= Eigen::VectorXd::Constant(lowerBound.size(), -0.01);
    upperBound += Eigen::VectorXd::Constant(upperBound.size(), 0.01);

    // instantiate the solver
    OsqpEigen::Solver solver;

    // settings
    solver.settings()->setWarmStart(true);

    // set the initial data of the QP solver
    solver.data()->setNumberOfVariables(MatDim);
    solver.data()->setNumberOfConstraints(EquDim);
    if(!solver.data()->setHessianMatrix(Hessian_sparse)) return falg;
    if(!solver.data()->setGradient(gradient)) return falg;
    if(!solver.data()->setLinearConstraintsMatrix(MatAeq_sparse)) return falg;
    if(!solver.data()->setLowerBound(lowerBound)) return falg;
    if(!solver.data()->setUpperBound(upperBound)) return falg;

    // 数都是正常的呀, 为什么会出现这种情况呢
    /***********************************************************************************
    ERROR in LDL_factor: Error in KKT matrix LDL factorization when computing the nonzero elements. The problem seems to be non-convex
    ERROR in osqp_setup: KKT matrix factorization.
    The problem seems to be non-convex.
    [OsqpEigen::Solver::initSolver] Unable to setup the workspace.
    ***********************************************************************************/
    // EigenCSV eigen_csv;
    // eigen_csv.WriteMatrix(Hessian_sparse, "/home/zwt/Documents/Hessian_sparse.csv");
    // std::cout << "Hessian_sparse" << std::endl;
    // eigen_csv.WriteMatrix(MatAeq_sparse, "/home/zwt/Documents/MatAeq_sparse.csv");
    // std::cout << "MatAeq_sparse" << std::endl;
    // eigen_csv.WriteVector(lowerBound, "/home/zwt/Documents/Bound.csv");
    // std::cout << "bound: " << std::endl;

    // instantiate the solver
    if(!solver.initSolver()) return falg;

    if(solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError) return falg;
    else {
        falg = true;
        QPoptx = solver.getSolution();
        std::cout << "[\033[32mOptimization\033[0m]: OSQP succeed" << std::endl;
        std::cout << "OSQP Solver result: " << QPoptx << std::endl;
    }

    OSQP_optx = QPoptx;

    return falg;
}

// NLopt Function
bool NonTrajOpt::NLoptSolve() {
    bool falg = true;
    nlopt::opt opt(nlopt::LD_LBFGS, OptNum);
    // ❗❗❗ NLoptobjection 必须返回 static double 类型的值 且该类型只能生命时候有，定义函数的时候就不能写 static 了
    opt.set_min_objective(NonTrajOpt::NLoptobjection, this);
    opt.set_xtol_rel(1e-4);
    opt.set_maxeval(nlopt_max_iteration_num_);
    opt.set_maxtime(nlopt_max_iteration_time_);
    double minf;
    std::vector<double> optx(OptNum);
    try {
        nlopt::result result = opt.optimize(optx, minf);
        std::cout << "[\033[32mOptimization\033[0m]: nlopt succeed" << std::endl;
        std::cout << "NLopt result: " << result << std::endl;
        std::cout << "NLopt minf: " << minf << std::endl;
    }
    catch(std::exception& e) {
        std::cout << "[\033[31mOptimization\033[0m]: nlopt exception" << std::endl;
        std::cout << "NLopt failed: " << e.what() << std::endl;
        falg = false; 
    }
    Non_optx = Eigen::Map<const Eigen::VectorXd>(optx.data(), optx.size());
    return falg;
}

// LBFGS Function
bool NonTrajOpt::LBFGSSolve() {
    bool flag = false;
    
    return flag;
}
// ##############################################################################################################
// Cost Functions 
// ##############################################################################################################
inline double NonTrajOpt::getSegPolyCost(const Eigen::VectorXd &coef, int idx) {
    double _cost = 0.0;
    _cost = 576.0 * coef(4) * coef(4) * T1(idx) +
           2880.0 * coef(4) * (coef(5)) * T2(idx) +
           4800.0 * coef(5) * coef(5) * T3(idx) +
           5760.0 * coef(4) * (coef(6)) * T3(idx) +
           21600.0 * coef(5) * (coef(6)) * T4(idx) +
           10080.0 * coef(4) * (coef(7)) * T4(idx) +
           25920.0 * coef(6) * coef(6) * T5(idx) +
           40320.0 * coef(5) * (coef(7)) * T5(idx) +
           100800.0 * coef(6) * (coef(7)) * T6(idx) +
           100800.0 * coef(7) * coef(7) * T7(idx);
    return _cost;
}
inline void NonTrajOpt::getSegPolyGradc(Eigen::VectorXd &coef,int idx){
    Eigen::VectorXd _gradc = Eigen::VectorXd::Zero(O);
    _gradc(4)=  1152*coef(4)*T1(idx) + 2880*coef(5)*T2(idx) + 5760*coef(6)*T3(idx) + 10080*coef(7)*T4(idx);
    _gradc(5)=  2880*coef(4)*T2(idx) + 9600*coef(5)*T3(idx) + 21600*coef(6)*T4(idx) + 40320*coef(7)*T5(idx);
    _gradc(6)=  5760*coef(4)*T3(idx) + 21600*coef(5)*T4(idx) + 51840*coef(6)*T5(idx) + 100800*coef(7)*T6(idx);
    _gradc(7)=  10080*coef(4)*T4(idx) + 40320*coef(5)*T5(idx) + 100800*coef(6)*T6(idx) + 201600*coef(7)*T7(idx);
    coef = _gradc;
}

inline double NonTrajOpt::getSegPolyGradt(const Eigen::VectorXd &coef,int idx){
    double _gradt = 0;
    _gradt = 576.0 * coef(4) * coef(4) + 
             5760.0 * coef(4) * coef(5)*T1(idx) +
             14400.0 * coef(5) * coef(5)*T2(idx) +
             17280.0 * coef(4) * coef(6)*T2(idx) +
             86400.0 * coef(5) * coef(6)*T3(idx) +
             40320.0 * coef(4) * coef(7)*T3(idx) +
             129600.0 * coef(6) * coef(6)*T4(idx) +
             201600.0 * coef(5) * coef(7)*T4(idx) +
             604800.0 * coef(6) * coef(7)*T5(idx) +
             705600.0 * coef(7) * coef(7)*T6(idx);
    return _gradt;
}

/***************************************************************************************************************
 * @description: calculate smoothness cost it's minimize the states such as velocity, acceleration, jerk, snap
 * @reference: https://gitee.com/hi-zwt/kinodynamicpath/blob/master/PolyOpt/smoothCost.m
 * @param {double} &Cost
 * @param {VectorXd} gradc
 * @param {VectorXd} gradt
 * @return {*}
 */
void NonTrajOpt::calcuSmoCost(double &Cost,Eigen::VectorXd gradc, Eigen::VectorXd gradt) {
    // smoCost = 0.0;
    // Eigen::VectorXd gradc = Eigen::VectorXd::Zero(MatDim);
    // Eigen::VectorXd gradt = Eigen::VectorXd::Zero(N);
    for (int idx = 0; idx < N; idx++) {
        Eigen::VectorXd xcoef = Optc.segment(idx*O*D, O);
        double xcost = getSegPolyCost(xcoef, idx);
        double xgradt = getSegPolyGradt(xcoef, idx);
        getSegPolyGradc(xcoef, idx);
        Eigen::VectorXd ycoef = Optc.segment(idx*O*D+O, O);
        double ycost = getSegPolyCost(ycoef, idx);
        double ygradt = getSegPolyGradt(ycoef, idx);
        getSegPolyGradc(ycoef, idx);
        Eigen::VectorXd qcoef = Optc.segment(idx*O*D+2*O, O);
        double qcost = getSegPolyCost(qcoef, idx);
        double qgradt = getSegPolyGradt(qcoef, idx);
        getSegPolyGradc(qcoef, idx);
        gradc.segment(idx*O*D, O) = xcoef;
        gradc.segment(idx*O*D+O, O) = ycoef;
        gradc.segment(idx*O*D+2*O, O) = qcoef*wq;
        gradt(idx) = xgradt + ygradt + qgradt*wq;
        Cost += xcost + ycost + qcost*wq;
    }
}

// ##############################################################################################################
inline Eigen::VectorXd NonTrajOpt::getTimeVec(const double t,const int row) {
    Eigen::VectorXd TimeVec = Eigen::VectorXd::Zero(O);
    for (int col = row; col < O; col++) {
        TimeVec(col) = factorial(col)/factorial(col-row)*std::pow(t,col-row);
    }
    return TimeVec;
}
/***************************************************************************************************************
 * @description: calculate obstacle cost by Eucilidean distance field
 * @reference: https://gitee.com/hi-zwt/kinodynamicpath/blob/master/PolyOpt/obstacleCost.m
 * @param {double} &Cost
 * @param {VectorXd} gradc
 * @param {VectorXd} gradt
 * @return {*}
 */
void NonTrajOpt::calcuObsCost(double &Cost,Eigen::VectorXd gradc, Eigen::VectorXd gradt) {
    for (int idx = 0; idx < Traj.getPieceNum(); idx++) {
        double vecCost = 0.0;
        Eigen::VectorXd xgrad = Eigen::VectorXd::Zero(O);
        Eigen::VectorXd ygrad = Eigen::VectorXd::Zero(O);
        double dt = Traj[idx].getDiscretet();
        for (int idy = 0; idy <= Traj[idx].getDiscreteNum(); idy++) {
            Eigen::Vector2d pos = Traj[idx].posVector.col(idy).head(2);
            Eigen::Vector2d vel = Traj[idx].velVector.col(idy).head(2);
            double dist; Eigen::Vector2d grad;
            edfmap.getDistGrad(pos,dist,grad);
            double Tdt = idy * dt;
            if (dist < dist_th) {
                vecCost += 1/std::pow(2*(dist),2) * vel.squaredNorm() * Tdt;
                Eigen::VectorXd vectime = getTimeVec(Tdt,0);
                xgrad += xgrad + grad(0)*vectime*(-2*std::pow(dist,3));
                ygrad += ygrad + grad(1)*vectime*(-2*std::pow(dist,3));
                // gradc.segment(idx*O*D, O) += 1.0 / (dist * dist); //* edfmap.getGrad(pos).transpose();
            }
        }
        Cost += vecCost;
        gradc.segment(idx*O*D, O) = xgrad;
        gradc.segment(idx*O*D+O, O) = ygrad;
    }
}

/***************************************************************************************************************
 * @description: calculate dynamic constraints cost
 * @reference: https://gitee.com/hi-zwt/kinodynamicpath/blob/master/PolyOpt/dynamicCost.m
 * @param {double} &Cost
 * @param {VectorXd} gradc
 * @param {VectorXd} gradt
 * @return {*}
 */
void NonTrajOpt::calcuDynCost(double &Cost,Eigen::VectorXd gradc, Eigen::VectorXd gradt) {
    for (int idx = 0; idx < Traj.getPieceNum(); idx++) {
        double velCost = 0.0,accCost = 0.0;
        double velgradt = 0.0,accgradt = 0.0;
        Eigen::VectorXd xvgrad = Eigen::VectorXd::Zero(O);
        Eigen::VectorXd yvgrad = Eigen::VectorXd::Zero(O);
        Eigen::VectorXd qvgrad = Eigen::VectorXd::Zero(O);
        Eigen::VectorXd xagrad = Eigen::VectorXd::Zero(O);
        Eigen::VectorXd yagrad = Eigen::VectorXd::Zero(O);
        Eigen::VectorXd qagrad = Eigen::VectorXd::Zero(O);
        double dt = Traj[idx].getDiscretet();
        for (int idy = 0; idy <= Traj[idx].getDiscreteNum(); idy++) {
            Eigen::Vector3d vel = Traj[idx].velVector.col(idy);
            Eigen::Vector3d acc = Traj[idx].accVector.col(idy);
            Eigen::Vector3d jer = Traj[idx].jerVector.col(idy);
            Eigen::Vector3d deltaVel(0.0,0.0,0.0);
            Eigen::Vector3d deltaAcc(0.0,0.0,0.0);
            double Tdt = idy * dt;
            Eigen::VectorXd vectime2 = getTimeVec(Tdt,1); // velocity order
            Eigen::VectorXd vectime3 = getTimeVec(Tdt,2); // acceleration order
            if ( vel(0) > pv_max ) {
                deltaVel(0) = vel(0) - pv_max;
            }else if ( vel(0) < -pv_max ) {
                deltaVel(0) = vel(0) + pv_max;
            } xvgrad += vectime2 * 2 * deltaVel(0);
            if ( vel(1) > pv_max ) {
                deltaVel(1) = vel(1) - pv_max;
            }else if ( vel(1) < -pv_max ) {
                deltaVel(1) = vel(1) + pv_max;
            } yvgrad += vectime2 * 2 * deltaVel(1);
            if ( vel(2) > wv_max ) {
                deltaVel(2) = vel(2) - wv_max;
            }else if ( vel(2) < -wv_max ) {
                deltaVel(2) = vel(2) + wv_max;
            } qvgrad += vectime2 * 2 * deltaVel(2);
            if ( acc(0) > pa_max ) {
                deltaAcc(0) = acc(0) - pa_max;
            }else if ( acc(0) < -pa_max ) {
                deltaAcc(0) = acc(0) + pa_max;
            } xagrad += vectime3 * 2 * deltaAcc(0);
            if ( acc(1) > pa_max ) {
                deltaAcc(1) = acc(1) - pa_max;
            }else if ( acc(1) < -pa_max ) {
                deltaAcc(1) = acc(1) + pa_max;
            } yagrad += vectime3 * 2 * deltaAcc(1);
            if ( acc(2) > wa_max ) {
                deltaAcc(2) = acc(2) - wa_max;
            }else if ( acc(2) < -wa_max ) {
                deltaAcc(2) = acc(2) + wa_max;
            } qagrad += vectime3 * 2 * deltaAcc(2);
            velCost += deltaVel.squaredNorm();
            accCost += deltaAcc.squaredNorm();
            velgradt += 2*deltaVel.dot(jer)*Tdt;
            accgradt += 2*deltaAcc.dot(jer)*Tdt;
        }
        Cost += velCost + accCost;
        gradc.segment(idx*O*D, O)       = xvgrad + xagrad;
        gradc.segment(idx*O*D+O, O)     = yvgrad + yagrad;
        gradc.segment(idx*O*D+2*O, O)   = qvgrad + qagrad;
        gradt(idx) = velgradt + accgradt;
    }
}

/***************************************************************************************************************
 * @description: calculate time cost
 * @reference: https://gitee.com/hi-zwt/kinodynamicpath/blob/master/PolyOpt/timeCost.m
 * @param {double} &Cost
 * @param {VectorXd} gradc
 * @param {VectorXd} gradt
 * @return {*}
 */
void NonTrajOpt::calcuTimCost(double &Cost,Eigen::VectorXd gradc, Eigen::VectorXd gradt) {
    Cost = T2.sum();
    gradc = Eigen::VectorXd::Zero(MatDim);
    gradt = 2*T2;
}
/***************************************************************************************************************
 * @description: calculate oval state constraints cost
 * @reference: https://gitee.com/hi-zwt/kinodynamicpath/blob/master/PolyOpt/ovalCost.m
 * @param {double} &Cost
 * @param {VectorXd} gradc
 * @param {VectorXd} gradt
 * @return {*}
 */
void NonTrajOpt::calcuOvaCost(double &Cost,Eigen::VectorXd gradc, Eigen::VectorXd gradt) {
    for (int idx = 0; idx < Traj.getPieceNum(); idx++) {
        Eigen::VectorXd xgrad = Eigen::VectorXd::Zero(O);
        Eigen::VectorXd ygrad = Eigen::VectorXd::Zero(O);
        Eigen::VectorXd qgrad = Eigen::VectorXd::Zero(O);
        double dt = Traj[idx].getDiscretet();
        for (int idy = 0; idy <= Traj[idx].getDiscreteNum(); idy++) {
            double yaw_q = Traj[idx].posVector(2,idy);
            Eigen::Vector3d vel = Traj[idx].velVector.col(idy);
            Eigen::Vector3d acc = Traj[idx].accVector.col(idy);
            double Tdt = idy * dt;
            Eigen::VectorXd vectime1 = getTimeVec(Tdt,0); // position order
            Eigen::VectorXd vectime2 = getTimeVec(Tdt,1); // velocity order
            Eigen::MatrixXd Rmatrix = Eigen::MatrixXd::Zero(2,2);
            double cosyaw = std::cos(yaw_q);
            double sinyaw = std::sin(yaw_q);
            // rotation matrix and state transform
            Rmatrix << cosyaw, -sinyaw,
                       sinyaw,  cosyaw;
            Eigen::Vector2d vel_B = Rmatrix * vel.head(2);
            Eigen::Vector2d acc_B = Rmatrix * acc.head(2);
            double vel_B2 = std::pow(vel_B(0),2)/ORIEN_VEL2 + std::pow(vel_B(1),2)/VERDIT_VEL2;
            if (vel_B2 > OVAL_TH) {
                Cost += vel_B2 - OVAL_TH;
                gradt(idx) += (2 * vel_B(1) * (acc_B(1) - cosyaw * vel(0) * vel(2) - sinyaw * vel(1) * vel(2)) / VERDIT_VEL2 +
                               2 * vel_B(0) * (acc_B(0) + cosyaw * vel(1) * vel(2) - sinyaw * vel(0) * vel(2)) / ORIEN_VEL2) * Tdt;
                xgrad += (2 * cosyaw * vel_B(0)/ORIEN_VEL2 - 2 * sinyaw * vel_B(1) / VERDIT_VEL2) * vectime2;
                ygrad += (2 * sinyaw * vel_B(0)/ORIEN_VEL2 + 2 * cosyaw * vel_B(1) / VERDIT_VEL2) * vectime2;
                qgrad += (2 * vel_B(0) * vel(1) / ORIEN_VEL2 - 2 * vel_B(1) * vel(0) / VERDIT_VEL2) * vectime1;
            }
        }
        gradc.segment(idx*O*D, O)       = xgrad;
        gradc.segment(idx*O*D+O, O)     = ygrad;
        gradc.segment(idx*O*D+2*O, O)   = qgrad;
    }
}

void NonTrajOpt::calcuOvaConstriants() {

}



// } // namespace NonTrajOptSpace

#endif // NONTRAJOPT_HPP_