/*
 * @Author: wentao zhang && zwt190315@163.com
 * @Date: 2023-07-05
 * @LastEditTime: 2023-07-26
 * @Description: Nonlinear Trajectory Optimization
 * @reference: 
 * 
 * Copyright (c) 2023 by wentao zhang, All Rights Reserved. 
 */

#ifndef NONTRAJOPT_HPP_
#define NONTRAJOPT_HPP_

#include <iostream>
#include <vector>
#include <angles/angles.h>
#include <Eigen/Eigen>
#include <Eigen/SparseQR>
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

//// optimization viables init
#define INIT_COEFFS 2.0
#define INIT_TIME_TAU -0.5
#define INIT_TIME 0.6
#define COEFFS_UPPER_BOUND 500.0
#define COEFFS_LOWER_BOUND -500.0
#define TIME_TAU_UPPER_BOUND 1.6
#define TIME_TAU_LOWER_BOUND -3.0


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
    double ref_vel,ref_ome; // reference linear velocity and angular velocity
    double pv_max,pa_max; // max linear velocity, acceleration,
    double wv_max,wa_max; // max angular velocity, acceleration,
    double dyn_rate;
    double ORIEN_VEL,VERDIT_VEL; // velocity
    double OVAL_TH; // oval cost threshold
    bool SMO_SWITCH;
    bool OBS_SWITCH;
    bool DYN_SWITCH;
    bool TIM_SWITCH;
    bool OVA_SWITCH;

    double coeff_bound;

    bool TIME_OPTIMIZATION;  // time optimization
    bool REDUCE_ORDER;
    bool BOUND_OPTIMIZATION; // bound optimization

    bool INIT_OPT_VALUE;   // init coeffs and time to  nonlinear optimization

    //&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
    int    nlopt_max_iteration_num_;  // stopping criteria that can be used
    double nlopt_max_iteration_time_;  // stopping criteria that can be used
    // int    nlopt_max_iteration_num_;  // stopping criteria that can be used
    // double nlopt_max_iteration_time_;  // stopping criteria that can be used
    // int    nlopt_max_iteration_num_;  // stopping criteria that can be used
    // double nlopt_max_iteration_time_;  // stopping criteria that can be used

    //esdf map&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
    double map_width;
    double map_height;
    double map_resolution;
    double mini_dist;

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
    public://##############################################################################################################
    int N;  // number of segments
    int O;  // default order is 7th (8 optimal values)
    int D;  // dimension of the trajectory
    int E;  // number of equality constraints(p v a j)
    int C;  // cost function order v=1 a=2 j=3 s=4; 4th order cost function

    public://##############################################################################################################
    /// @brief: Nonlinear Optimization
    // &&&&&&&&&&&&&&&&& // full odrer Ax=b solve x
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
    private://##############################################################################################################
    // cost, gradient and weight 
    Eigen::VectorXd smogd;
    Eigen::VectorXd obsgd;
    Eigen::VectorXd dyngd;
    Eigen::VectorXd timgd;
    Eigen::VectorXd ovagd;

    double smoCost, obsCost, dynCost, timCost, ovaCost;

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
    double ref_vel,ref_ome; // reference linear velocity and angular velocity
    double pv_max,pa_max,wv_max,wa_max; // max velocity, acceleration,
    double ORIEN_VEL,VERDIT_VEL; // velocity
    double ORIEN_VEL2,VERDIT_VEL2; // velocity^2
    double OVAL_TH; // oval cost threshold

    double coeff_bound;
    int    nlopt_max_iteration_num_;  // stopping criteria that can be used
    double nlopt_max_iteration_time_;  // stopping criteria that can be used

    bool SMO_SWITCH;
    bool OBS_SWITCH;
    bool DYN_SWITCH;
    bool TIM_SWITCH;
    bool OVA_SWITCH;

    bool INIT_OPT_VALUE;   // use search to init nonlinear optimization
    bool BOUND_OPTIMIZATION; // bound optimization

    /// @brief: Quadratic Programming
    /// @reference: argmin 1/2 x^T Q x + f^T x ; s.t. Aeq x = beq
    Eigen::MatrixXd MatQ;   // QP problem Q
    Eigen::VectorXd QPx;    // QP problem x
    Eigen::MatrixXd MatAeq; // QP problem Aeq
    Eigen::VectorXd Vecbeq; // QP problem beq
    
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
    public:
    Eigen::Matrix3Xd Waypoints;
    Eigen::Matrix<double, 3, 4> StartStates; 
    Eigen::Matrix<double, 3, 4> EndStates;
    Eigen::VectorXd initT; // initial time
    Eigen::VectorXi discNums; // discretization number of each segment
    Eigen::VectorXd discDist; // discretization distance of each segment

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
    private:
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
    void calcuSmoCost(double &Cost,Eigen::VectorXd &gradc, Eigen::VectorXd &gradt);       // calculate smoothness cost
    void calcuObsCost(double &Cost,Eigen::VectorXd &gradc, Eigen::VectorXd &gradt);       // calculate obstacle cost
    void calcuDynCost(double &Cost,Eigen::VectorXd &gradc, Eigen::VectorXd &gradt);       // calculate dynamic cost
    void calcuTimCost(double &Cost,Eigen::VectorXd &gradc, Eigen::VectorXd &gradt);       // calculate time cost
    void calcuOvaCost(double &Cost,Eigen::VectorXd &gradc, Eigen::VectorXd &gradt);       // calculate overall cost

    // void combineCost(double &Cost,Eigen::VectorXd gradc, Eigen::VectorXd gradt);

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
    ////Debug: &&&&&&&&&&&&&&
    Eigen::MatrixXd gdsmotc; // gradient of smoothness cost
    Eigen::MatrixXd gdobstc; // gradient of obstacle cost
    Eigen::MatrixXd gddyntc; // gradient of dynamic cost
    Eigen::MatrixXd gdovatc; // gradient of time cost
    Eigen::MatrixXd gdsmocostgrad;
    Eigen::MatrixXd gdobstimemat;

    std::vector<int> isOpt; // isOpt[i]  表示第n个变量是优化变量
    std::vector<int> noOpt; // noOpt[i]  表示第n个系数不是优化变量
    std::vector<bool> optFlag; // optFlag[i]  表示第n个系数是否是优化变量

    NonTrajOpt(/* args */) = default;
    ~NonTrajOpt() = default;

    /* main API */
    void setParam(ros::NodeHandle &nh,OptParas &paras);
    void showParam();
    

    bool OSQPSolve();   //TUDO： 革命尚未成功，同志仍需努力
    bool NLoptSolve();
    bool LBFGSSolve();

    // fixed parameters for every optimization 
    void initParameter(const OptParas &paras);

    // every time optimization reset parameters
    void reset(const int &pieceNum);
    // init waypoints and start/end states  for polynomial trajectory optimization
    inline void initWaypoints(const Eigen::Matrix3Xd &_waypoints, const Eigen::VectorXd &_initT, 
                       const Eigen::Matrix<double, 3, 4> &_startStates, const Eigen::Matrix<double, 3, 4> &_endStates);
    bool setWaypoints(const std::vector<Eigen::Vector3d> &_waypoints, 
                       const Eigen::Matrix<double, 3, 3> &_startStates, const Eigen::Matrix<double, 3, 3> &_endStates);
    bool pushWaypoints(const std::vector<Eigen::Vector3d> &_waypoints, const std::vector<double> &_initT,
                       const Eigen::Matrix<double, 3, 3> &_startStates, const Eigen::Matrix<double, 3, 3> &_endStates);
    inline void setEDFMap( const double map_size_x, const double map_size_y, 
                    const double map_resolution, const double mini_dist);
    inline bool readEDFMap(const std::string &filename,const int kernel_size); // read edf map
    bool updateEDFMap(const cv::Mat &img); // update edf map
    bool updateEDFMap(const cv::Mat* img); // update edf map
    
    inline void getReduceOrder(Eigen::VectorXd &_vec);  // reduce order vector
    inline void getRawEquation(Eigen::VectorXd &_vec);  // raw equation vector
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
    private:
    void calcuOvaConstriants();                       // calculate overall constraints

    static double NLoptobjection(const std::vector<double>& optx, std::vector<double>& grad,void* func_data);
    static double LBFGSobjection(void* ptrObj,const double* optx,double* grad,const int n);

};

// } // namespace NonTrajOptSpace

#endif // NONTRAJOPT_HPP_