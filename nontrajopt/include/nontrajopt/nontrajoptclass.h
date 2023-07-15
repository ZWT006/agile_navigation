/*
 * @Author: wentao zhang && zwt190315@163.com
 * @Date: 2023-07-05
 * @LastEditTime: 2023-07-14
 * @Description: Nonlinear Trajectory Optimization
 * @reference: 
 * 
 * Copyright (c) 2023 by wentao zhang, All Rights Reserved. 
 */

#ifndef NONTRAJOPTCLASS_HPP_
#define NONTRAJOPTCLASS_HPP_

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
    // private://##############################################################################################################
    public://##############################################################################################################
    int N;  // number of segments
    int O;  // default order is 7th (8 optimal values)
    int D;  // dimension of the trajectory
    int E;  // number of equality constraints(p v a j)
    int C;  // cost function order v=1 a=2 j=3 s=4; 4th order cost function

    private://##############################################################################################################
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



    // cost, gradient and weight 
    Eigen::VectorXd smogd;
    Eigen::VectorXd obsgd;
    Eigen::VectorXd dyngd;
    Eigen::VectorXd timgd;
    Eigen::VectorXd ovagd;

    double smoCost, obsCost, dynCost, timCost, ovaCost;

    public:
    bool TIME_OPTIMIZATION = true;  // time optimization
    // if true Optx = [Optc; Optt] else Optx = Optc
    // Vect = initT; or Vect = exp(Optt)
    bool REDUCE_ORDER = true;       // solve Ax=b to reduce order
    // if true Vecx = Vecb/MatA else Vecx = optc
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
    // EDFMap edfmap;
    // Trajectory Traj;
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
    public:
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
    int iter_num = 0;

    //// Debug 
    EDFMap edfmap;
    Trajectory Traj;


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

    inline void setEDFMap( const double map_size_x, const double map_size_y, 
                    const double map_resolution, const double mini_dist);
    inline bool updateEDFMap(const cv::Mat &img); // update edf map
    inline bool readEDFMap(const std::string &filename,const int kernel_size); // read edf map

    // key functions &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
    inline void updateOptVars(const Eigen::VectorXd &_optvar);  // update optimization variables
    inline void updateTime() ;
    // for Nonlinear Optimization
    inline void updateOptAxb();                              // update trajectory Matrix by Optx
    // for QP problem
    inline void updateAeqbeq();    // update MatA, Vecb by time initT  
    inline void updateMatQ();      // update MatQ by time initT
    // for Trajectory Traj
    inline void updateTraj();      // update Traj by Vecx and Vect
    
    void calcuOvaConstriants();                       // calculate overall constraints

    static double NLoptobjection(const std::vector<double>& optx, std::vector<double>& grad,void* func_data);
    static double LBFGSobjection(void* ptrObj,const double* optx,double* grad,const int n);

    bool OSQPSolve();   //TUDO： 革命尚未成功，同志仍需努力
    bool NLoptSolve();
    bool LBFGSSolve();
};

// } // namespace NonTrajOptSpace

#endif // NONTRAJOPT_HPP_