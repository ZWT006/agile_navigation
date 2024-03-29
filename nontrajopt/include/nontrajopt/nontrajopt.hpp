/*
 * @Author: wentao zhang && zwt190315@163.com
 * @Date: 2023-07-05
 * @LastEditTime: 2023-11-22
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

#include <ros/ros.h>

//// optimization viables init
#define INIT_COEFFS 2.0
#define INIT_TIME_TAU -0.5
#define INIT_TIME 0.6
#define COEFFS_UPPER_BOUND 500.0
#define COEFFS_LOWER_BOUND -500.0
#define TIME_TAU_UPPER_BOUND 1.6
#define TIME_TAU_LOWER_BOUND -3.0
#define EQ_OFFSET 24 // Ax=b中的A矩阵构建增广矩阵的时候，插入单位阵的偏移量；使其插入位置在线性相关列对应的行


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
    double nlopt_ftol_rel_;  // function stopping criteria that can be used
    double nlopt_xtol_rel_;  // variable stopping criteria that can be used
    OptParas(){};
    ~OptParas(){};
};

/**
 * @description: calculate M-P inverse
 * @reference: https://blog.csdn.net/qq_34935373/article/details/107560470
 * @param {MatrixXd } A
 * @return {MatrixXd } X
 */
Eigen::MatrixXd Eigenpinv(Eigen::MatrixXd  A)
{
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);
    double  pinvtoler = 1.e-8; //tolerance
    int row = A.rows();
    int col = A.cols();
    int k = std::min(row,col);
    Eigen::MatrixXd X = Eigen::MatrixXd::Zero(col,row);
    Eigen::MatrixXd singularValues_inv = svd.singularValues();//奇异值
    Eigen::MatrixXd singularValues_inv_mat = Eigen::MatrixXd::Zero(col, row);
    for (long i = 0; i<k; ++i) {
        if (singularValues_inv(i) > pinvtoler)
            singularValues_inv(i) = 1.0 / singularValues_inv(i);
        else singularValues_inv(i) = 0;
    }
    for (long i = 0; i < k; ++i) 
    {
        singularValues_inv_mat(i, i) = singularValues_inv(i);
    }
    X=(svd.matrixV())*(singularValues_inv_mat)*(svd.matrixU().transpose());
 
    return X;
 
}

/*****************************************************************************************************
//// @brief Nonlinear Trajectory Optimization
//// @reference: https://github.com/ZJU-FAST-Lab/GCOPTER/blob/main/gcopter/include/gcopter/minco.hpp
//// @reference: https://github.com/HKUST-Aerial-Robotics/Fast-Planner/tree/master/fast_planner/bspline_opt
//// @reference: NLopt : https://nlopt.readthedocs.io/en/latest/
*/
class NonTrajOpt
{
    // private://##############################################################################################################
    public:////Debug: public ##############################################################################################################
    int N;  // number of segments
    int O;  // default order is 7th (8 optimal values)
    int D;  // dimension of the trajectory
    int E;  // number of equality constraints(p v a j)
    int C;  // cost function order v=1 a=2 j=3 s=4; 4th order cost function

    private://##############################################################################################################
    /// @brief: Nonlinear Optimization
    // &&&&&&&&&&&&&&&&& // full odrer Ax=b solve x
    public: ////Debug: public ##############################################################################################################
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
    double ref_vel,ref_ome; // reference linear velocity and angular velocity
    double pv_max,pa_max,wv_max,wa_max; // max velocity, acceleration,
    double ORIEN_VEL,VERDIT_VEL; // velocity
    double ORIEN_VEL2,VERDIT_VEL2; // velocity^2
    double OVAL_TH; // oval cost threshold

    int    nlopt_max_iteration_num_;  // stopping criteria that can be used
    double nlopt_max_iteration_time_;  // stopping criteria that can be used
    double nlopt_ftol_rel_;  // function stopping criteria that can be used
    double nlopt_xtol_rel_;  // variable stopping criteria that can be used

    bool SMO_SWITCH;
    bool OBS_SWITCH;
    bool DYN_SWITCH;
    bool TIM_SWITCH;
    bool OVA_SWITCH;

    bool INIT_OPT_VALUE;   // use search to init nonlinear optimization
    bool BOUND_OPTIMIZATION; // bound optimization

    /// @brief: Quadratic Programming
    /// @reference: argmin 1/2 x^T Q x + f^T x ; s.t. Aeq x = beq
    public:
    Eigen::MatrixXd MatQ;   // QP problem Q
    Eigen::VectorXd QPx;    // QP problem x
    Eigen::MatrixXd MatAeq; // QP problem Aeq
    Eigen::VectorXd Vecbeq; // QP problem beq
    ////&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
    Eigen::MatrixXd MatkQ;  // QP problem Q for [x,y,q] 
    Eigen::MatrixXd MatAeqx; // QP problem Aeq for x
    Eigen::MatrixXd MatAeqy; // QP problem Aeq for y
    Eigen::MatrixXd MatAeqq; // QP problem Aeq for q
    
    Eigen::VectorXd Vecbeqx; // QP problem beq for x
    Eigen::VectorXd Vecbeqy; // QP problem beq for y
    Eigen::VectorXd Vecbeqq; // QP problem beq for q
    Eigen::VectorXd QPxDiv; // QP problem x for [x,y,q]
    Eigen::VectorXd QPyDiv; // QP problem y for [x,y,q]
    Eigen::VectorXd QPqDiv; // QP problem q for [x,y,q]
    int EquDimDiv; // dimension of equality constraints for [x,y,q]
    int MatDimDiv; // dimension of matrix A for [x,y,q]

    public:
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
    void setParam(ros::NodeHandle &nh);
    bool updateEDFMap(const cv::Mat &img); // update edf map

    // fixed parameters for every optimization 
    void initParameter(const OptParas &paras);

    // every time optimization reset parameters
    void reset(const int &pieceNum);
    // init waypoints and start/end states  for polynomial trajectory optimization
    inline void initWaypoints(const Eigen::Matrix3Xd &_waypoints, const Eigen::VectorXd &_initT, 
                       const Eigen::Matrix<double, 3, 4> &_startStates, const Eigen::Matrix<double, 3, 4> &_endStates);
    bool setWaypoints(const std::vector<Eigen::Vector3d> &_waypoints, 
                       const Eigen::Matrix<double, 3, 3> &_startStates, const Eigen::Matrix<double, 3, 3> &_endStates);
    inline void getReduceOrder(Eigen::VectorXd &_vec);  // reduce order vector
    inline void getRawEquation(Eigen::VectorXd &_vec);  // raw equation vector

    inline void setEDFMap( const double map_size_x, const double map_size_y, 
                    const double map_resolution, const double mini_dist);
    inline bool readEDFMap(const std::string &filename,const int kernel_size); // read edf map

    // key functions &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
    inline void updateOptVars(const Eigen::VectorXd &_optvar);  // update optimization variables
    inline void updateTime() ;
    // for Nonlinear Optimization
    inline void updateOptAxb();                              // update trajectory Matrix by Optx
    // for QP problem
    inline void updateAeqbeq();    // update MatA, Vecb by time initT  
    inline void updateMatQ();      // update MatQ by time initT
    inline void updateAeqbeqDiv(); // update MatAeq, Vecbeq by time initT as [x,y,q]
    inline void updateMatQDiv();   // update MatQ by time initT as [x,y,q]
    // for Trajectory Traj
    inline void updateTraj();      // update Traj by Vecx and Vect
    
    void calcuOvaConstriants();                       // calculate overall constraints

    static double NLoptobjection(const std::vector<double>& optx, std::vector<double>& grad,void* func_data);
    static double LBFGSobjection(void* ptrObj,const double* optx,double* grad,const int n);

    bool OSQPSolve();   //TUDO： 革命尚未成功，同志仍需努力
    bool OSQPSolveDiv();
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

    SMO_SWITCH = paras.SMO_SWITCH;
    OBS_SWITCH = paras.OBS_SWITCH;
    DYN_SWITCH = paras.DYN_SWITCH;
    TIM_SWITCH = paras.TIM_SWITCH;
    OVA_SWITCH = paras.OVA_SWITCH;
    
    wq         = paras.wq; // weight of state.q coefficients 
    dist_th    = paras.dist_th; // distance threshold of obstacle   
    ref_vel    = paras.ref_vel; // reference linear velocity
    ref_ome    = paras.ref_ome; // reference angular velocity
    pv_max     = paras.pv_max * paras.dyn_rate;
    pa_max     = paras.pa_max * paras.dyn_rate;
    wv_max     = paras.wv_max * paras.dyn_rate;
    wa_max     = paras.wa_max * paras.dyn_rate; // max velocity, acceleration,
    discredist = paras.discredist; // distance discretization
    OVAL_TH    = paras.OVAL_TH; // oval cost threshold
    ORIEN_VEL  = paras.ORIEN_VEL;
    VERDIT_VEL = paras.VERDIT_VEL; // velocity
    ORIEN_VEL2 = ORIEN_VEL * ORIEN_VEL; // velocity orientaion
    VERDIT_VEL2 = VERDIT_VEL * VERDIT_VEL; // velocity vertical

    TIME_OPTIMIZATION   = paras.TIME_OPTIMIZATION;  // time optimization
    REDUCE_ORDER        = paras.REDUCE_ORDER;
    BOUND_OPTIMIZATION  = paras.BOUND_OPTIMIZATION; // bound optimization

    INIT_OPT_VALUE = paras.INIT_OPT_VALUE;   // use search to init nonlinear optimization

    //&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
    // Optimization Solver Parameters
    nlopt_max_iteration_num_ = paras.nlopt_max_iteration_num_;
    nlopt_max_iteration_time_ = paras.nlopt_max_iteration_time_;
    nlopt_ftol_rel_ = paras.nlopt_ftol_rel_;
    nlopt_xtol_rel_ = paras.nlopt_xtol_rel_;

    O = 8; // 7th = 8 coefficients
    D = 3; // [x,y,q]
    E = 4; // [p,v,a,j]
    C = 4; // cost function order
}

/*****************************************************************************************************
 * @description: 
 * @reference: 
 * @param {Eigen::Matrix3Xd} _waypoints column vector of waypoints
 * @param {Eigen::VectorXd} _initT
 * @param {Eigen::Matrix<double, 3, 4>} _startStates pos;vel;acc;jer
 * @param {Eigen::Matrix<double, 3, 4>} _endStates  pos;vel;acc;jer
 * @return {*}
 */
inline void NonTrajOpt::initWaypoints(const Eigen::Matrix3Xd &_waypoints, const Eigen::VectorXd &_initT, 
    const Eigen::Matrix<double, 3, 4> &_startStates, const Eigen::Matrix<double, 3, 4> &_endStates) {
    Waypoints = _waypoints;
    StartStates = _startStates;
    EndStates = _endStates;
    initT = _initT;
    Vect = initT;
    double dist = 0.0;
    for (int i = 0; i < N; i++) {
        Eigen::Vector2d xypos_s = Waypoints.col(i).head(2);
        Eigen::Vector2d xypos_e = Waypoints.col(i+1).head(2);
        Eigen::Vector2d xypos = xypos_e - xypos_s;
        dist = xypos.norm();
        discNums(i) = std::ceil(dist/discredist);
        discDist(i) = dist;
    }
    updateTime();   
}
/*****************************************************************************************************
 * @description: set Waypoints
 * @reference: 
 * @param {std::vector<Eigen::Vector3d>} _waypoints vector of waypoints pos
 * @param {Eigen::Matrix<double, 3, 3>} _startStates pos;vel;acc
 * @param {Eigen::Matrix<double, 3, 3>} _endStates  pos;vel;acc
 * @return {bool} flag of success
 */
bool NonTrajOpt::setWaypoints(const std::vector<Eigen::Vector3d> &_waypoints, 
    const Eigen::Matrix<double, 3, 3> &_startStates, const Eigen::Matrix<double, 3, 3> &_endStates) {

    if ( _waypoints.size() - N != 1 ) {
        return false;
    }
    //// 注意 waypoints 的数量比 轨迹段 N 多 1 ##############################################################
    Eigen::Matrix3Xd Initwaypoints = Eigen::Matrix3Xd::Zero(3, _waypoints.size());
    Eigen::VectorXd Vecyaw = Eigen::VectorXd::Zero(_waypoints.size());
    Eigen::VectorXd InitT = Eigen::VectorXd::Zero(N);
    Eigen::Matrix<double, 3, 4> StartStates = Eigen::Matrix<double, 3, 4>::Zero();
    StartStates.block(0,0,3,3) = _startStates;
    Eigen::Matrix<double, 3, 4> EndStates = Eigen::Matrix<double, 3, 4>::Zero();
    _endStates;
    for (int idx = 0; idx < N + 1 ; idx++) {
        Initwaypoints.col(idx) = _waypoints[idx];
    //// Step 0: yaw 角归一化到 [-pi pi] ##################################################################
        Vecyaw(idx) = angles::normalize_angle(_waypoints[idx](2));
    }
    //// Step 1: yaw 角的平滑处理 缩小使转向角变化
    Eigen::VectorXd VecyawSmooth = Eigen::VectorXd::Zero(_waypoints.size());
    for (int idx = 1; idx < N ; idx++) {
        VecyawSmooth(idx) = angles::shortest_angular_distance(Vecyaw(idx),Vecyaw(idx+1))/2.0 + Vecyaw(idx);
    }
    VecyawSmooth(0) = Vecyaw(0); VecyawSmooth(N-1) = Vecyaw(N-1);
    Vecyaw = VecyawSmooth;
    //// Step 2: reference time 参考优化时间计算 ###########################################################
    for (int idx = 0; idx < N; idx++) {
        Eigen::Vector2d xypos_s = Initwaypoints.col(idx).head(2);
        Eigen::Vector2d xypos_e = Initwaypoints.col(idx+1).head(2);
        Eigen::Vector2d xypos = xypos_e - xypos_s;
        double xy_dist = xypos.norm();
        double q_dist = angles::shortest_angular_distance(Vecyaw(idx),Vecyaw(idx+1));
        InitT(idx) = std::max(xy_dist/ref_vel,q_dist/ref_ome);
    }
    //// Step 3: yaw 角的数值连续性处理 用于数值优化 #########################################################
    for (int idx = 1; idx <= N; idx++) { ////Note: 这里的第 N 个点会不会有问题? 比原定的点偏离? 应该不会吧?
        VecyawSmooth(idx) = VecyawSmooth(idx-1) + angles::shortest_angular_distance(Vecyaw(idx-1),Vecyaw(idx));
    }
    VecyawSmooth(0) = Vecyaw(0); 
    for (int idx = 0; idx <= N; idx++) {
        Initwaypoints(2,idx) = VecyawSmooth(idx);
    }
    //// Step 4: init waypoints and start/end states for polynomial trajectory optimization
    initWaypoints(Initwaypoints,InitT,StartStates,EndStates);
    return true;
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

inline void NonTrajOpt::getRawEquation(Eigen::VectorXd &_vec) {
    Eigen::VectorXd re_vec = Eigen::VectorXd::Zero(EquDim);
    // update OptDof equality constraints
    int equ_num = 0;
    for (int idx = 0; idx < MatDim && equ_num < EquDim; idx++) {
        if (!optFlag[idx]) { // true is optimization variable
            re_vec(equ_num) = _vec(idx);
            equ_num++;
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

inline void NonTrajOpt::setEDFMap( const double map_size_x, const double map_size_y, 
                    const double map_resolution, const double mini_dist) {
    edfmap.setMapParam(map_size_x, map_size_y, map_resolution, mini_dist);
}

inline bool NonTrajOpt::updateEDFMap(const cv::Mat &img) {
    if (img.empty()) {
        std::cout << "edf map is empty" << std::endl;
        return false;
    }
    edfmap.setMap(img);
    return true;
}
inline bool NonTrajOpt::readEDFMap(const std::string &filename,const int kernel_size) {
    bool flag = false;
    if (filename.empty()) {
        std::cout << "edf map is empty" << std::endl;
        return flag;
    }
    flag = edfmap.readMap(filename,kernel_size);
    return flag;
}

// ##############################################################################################################
// optimization key functions ###################################################################################
// ##############################################################################################################

inline void NonTrajOpt::reset(const int &pieceNum) {
    N = pieceNum;
    MatDim = N * O * D;
    EquDim = 2*E*D + E*(N-1)*D + (N-1)*D;
    MatDimDiv = N * O;
    EquDimDiv = 2*E + E*(N-1) + (N-1);
    // opt 问题的自由度 整个自由度 减去 等式约束 (waypoints连续性约束+waypoints固定点约束)
    OptDof = MatDim - EquDim;
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
    Waypoints.resize(D, N+1);
    StartStates.setZero();
    EndStates.setZero();
    initT.resize(N);
    discNums.resize(N);
    discDist.resize(N);
    // QP problem resize &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
    MatQ.resize(MatDim, MatDim);
    QPx.resize(MatDim);
    MatAeq.resize(EquDim, MatDim);
    Vecbeq.resize(EquDim);

    MatkQ.resize(MatDimDiv, MatDimDiv);
    MatAeqx.resize(EquDimDiv, MatDimDiv);
    MatAeqy.resize(EquDimDiv, MatDimDiv);
    MatAeqq.resize(EquDimDiv, MatDimDiv);
    Vecbeqx.resize(EquDimDiv);
    Vecbeqy.resize(EquDimDiv);
    Vecbeqq.resize(EquDimDiv);

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
    // for (int id_seg = 0; id_seg < N  && dof_num < OptDof; id_seg++) { // N segments
    //     for (int id_dim = 0; id_dim < D && dof_num < OptDof; id_dim++) { // 3 dimensions
    //         for (int id_ord = E +1 ; id_ord < O && dof_num < OptDof; id_ord++) { // 5~7 orders
    //             int id_num = id_seg*O*D + id_dim*O + id_ord;
    //             optFlag[id_num] = true;
    //             dof_num++;
    //         }
    //     }
    // }
    for (int id_seg = 0; id_seg < N-1 ; id_seg++) { // N segments 
        for (int id_dim = 0; id_dim < D ; id_dim++) { // 3 dimensions
            for (int id_ord = E +1 ; id_ord < O ; id_ord++) { // 5~7 orders
                int id_num = id_seg*O*D + id_dim*O + id_ord + EQ_OFFSET;
                optFlag[id_num] = true;
                dof_num++;
            }
        }
    }
    
    // std::cout << "OptDof = " << OptDof << "; DofNum = " << dof_num << std::endl;
    // for (bool b : optFlag) {
    //     std::cout << b << " ";  // 这里会输出 1 或 0，而不是 true 或 false
    // }
    // 迭代次数清零
    iter_num = 0;
}

bool first_Abeq = true;
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

    // std::cout << "beq_wp.rows() : " << beq_wp.rows() << " beq_wp.cols() : " << beq_wp.cols() << std::endl;
    // std::cout << "waypoints: " << std::endl;
    // std::cout << Waypoints << std::endl;

    for (int k = 0; k < N-1; k++) {
        double ti = Vect(k);
        Eigen::MatrixXd coeff = getCoeffCons(ti,O,1);
        // std::cout << "coeff.rows() : " << coeff.rows() << " coeff.cols() : " << coeff.cols() << std::endl;
        Eigen::MatrixXd Matp = getCoeffCons(ti);
        // std::cout << "Matp.rows() : " << Matp.rows() << " Matp.cols() : " << Matp.cols() << std::endl;
        for(int i = 0; i < D; i++) {
            int KDi = k*D+i;
            // waypoints pose constraints
            // std::cout << "(E+1)*KDi = " << (E+1)*KDi << std::endl;
            // std::cout << "(i,k+1) =(" << i << " " << k+1 <<")"<< std::endl; 
            // std::cout << "waypoints(i,k+1)    = " << Waypoints(i,k+1);
            // std::cout << "beq_wp((E+1)*KDi,1) = " << beq_wp((E+1)*KDi,0) << std::endl;
            
            beq_wp((E+1)*KDi,0) = Waypoints(i,k+1);
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

    //// update OptDof equality constraints
    int dof_num = 0; //NOTE: 感觉这里的 第二和第三个循环的顺序不影响问题求解 但是不太好
    for (int id_seg = 0; id_seg < N-1 ; id_seg++) { // N segments
        for (int id_dim = 0; id_dim < D ; id_dim++) { // 3 dimensions
            for (int id_ord = E +1 ; id_ord < O ; id_ord++) { // 5~7 orders
                int id_num = id_seg*O*D + id_dim*O + id_ord + EQ_OFFSET;
                Eigen::VectorXd tempRow = Eigen::VectorXd::Zero(MatDim);
                tempRow(id_num) = 1.0;
                TempAeqRE.row(id_num) = tempRow.transpose();
                TempbeqRE(id_num) = Optx(dof_num);
                dof_num++;
            }
        }
    }
    
    // std::cout << "OptDof = " << OptDof << "; DofNum = " << dof_num << std::endl;

    int equ_num = 0;
    for (int idx = 0; idx < MatDim && equ_num < EquDim; idx++) {
        if (!optFlag[idx]) {
            TempAeqRE.row(idx) = TempAeq.row(equ_num);
            TempbeqRE(idx) = Tempbeq(equ_num);
            equ_num++;
        }
    }
    // std::cout << "EquDim = " << EquDim << "; equ num = " << equ_num << std::endl;
    // if (first_Abeq) {
    //     std::cout << "Write TempAeqRE.csv" << std::endl;
    //     std::cout << "Time Vector raw :" << Optt.transpose() << std::endl;
    //     std::cout << "Time Vector exp :" << Vect.transpose() << std::endl;
    //     eigenCSV.WriteMatrix(TempAeqRE,"/home/zwt/Documents/TempAeqRE.csv");
    //     eigenCSV.WriteVector(TempbeqRE,"/home/zwt/Documents/TempbeqRE.csv");
    //     first_Abeq = false;
    // }

    // Debug &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
    // std::cout << "TempAeq sizes: " << TempAeq.rows() << " " << TempAeq.cols() << std::endl;
    // std::cout << "TempAeq rank: " << TempAeq.fullPivLu().rank() << std::endl;
    // std::cout << "TempAeqRE sizes: " << TempAeqRE.rows() << " " << TempAeq.cols() << std::endl;
    // std::cout << "TempAeqRE rank: " << TempAeqRE.fullPivLu().rank() << std::endl;
    // 这里可以使用 fullPivLu() 类来求解，也可以使用 colPivHouseholderQr() 类来求解
    // 但是 GCOPTER 中使用的就是自己写的 BandedSystem 类来求解  可能是因为稀疏矩阵求解效率的原因吧 可与对比对比
    // 求解线性方程组 Ax=b &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
    // 线性方程组能够 正常求解且与MATLAB结果一致
    ////TODO: 这里求解线性方程组可能会有问题
    int rank = TempAeqRE.fullPivLu().rank();
    std::cout << "Aeq Rank: " << rank << "; MatDim: " << MatDim << std::endl;
    // 将 TempAeq 和 Tempbeq 赋值给 MatABS 和 Vecb
    MatA = TempAeqRE;
    Vecb = TempbeqRE;
    //// 最小二乘求解线性方程组 &&&&&&&&&&&&&&&&&&&&&&&&&&&&&
    // Vecx = Eigenpinv(TempAeqRE)*Vecb;
    Vecx = TempAeqRE.fullPivLu().solve(TempbeqRE);
    // std::cout << "Max element of TempAeqRE: " << TempAeqRE.maxCoeff() << std::endl;
    // std::cout << "Min element of TempAeqRE: " << TempAeqRE.minCoeff() << std::endl;
    // Eigen::MatrixXd invAeq = Eigenpinv(TempAeqRE);
    // Vecx = invAeq*Vecb;
    // std::cout << "Max element of invAeq: " << invAeq.maxCoeff() << std::endl;
    // std::cout << "Min element of invAeq: " << invAeq.minCoeff() << std::endl;
    // Eigen::SparseQR<Eigen::SparseMatrix<double>, Eigen::COLAMDOrdering<int>> solver;
    // 设置求解器的参数（可选）
    // solver.setPivotThreshold(1e-6); // 设置主元选取阈值，默认为1e-6
    // if (rank < MatDim) {
    //     // std::cout << YELLOW << "Write TempAeqRANK.csv" << RESET << std::endl;
    //     // std::cout << "Time Vector raw :" << Optt.transpose() << std::endl;
    //     // std::cout << "Time Vector exp :" << Vect.transpose() << std::endl;
    //     // eigenCSV.WriteMatrix(TempAeqRE,"/home/zwt/Documents/TempAeqRANK.csv");
    //     // eigenCSV.WriteVector(TempbeqRE,"/home/zwt/Documents/TempbeqRANK.csv");
    //     // std::cout << std::endl;
    //     std::cout << YELLOW << "God damn it! TempAeqRE rand :" << rank <<" < " << MatDim << " is singular!" << RESET << std::endl;
    //     // Vecx = TempAeqRE.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(TempbeqRE);
    //     Vecx = TempAeqRE.fullPivLu().solve(TempbeqRE);
    //     // Eigen::SparseMatrix<double> A = TempAeqRE.sparseView();
    //     // solver.compute(A);
    //     // if (solver.info() != Eigen::Success) {
    //     //     std::cout << "sad SparseQR faild ....." << std::endl;
    //     // }
    //     // Vecx = solver.solve(TempbeqRE); // 使用求解器求解线性方程组
    // }
    // else {
    //     Vecx = TempAeqRE.fullPivLu().solve(TempbeqRE);
    // // std::cout << "Vecx sizes: " << Vecx.rows() << " " << Vecx.cols() << std::endl;
    // // std::cout << "Vecx: " << Vecx.transpose() << std::endl;
    // }
}

inline void NonTrajOpt::updateTraj() {
    std::vector<CoefficientMat> coeffMats;
    // std::cout << "Vecx sizes: " << Vecx.rows() << " " << Vecx.cols() << std::endl;
    for (int i = 0; i < N; i++){
        CoefficientMat coeffMat;
        coeffMat << Vecx.segment(i*O*D, O).transpose(),
                    Vecx.segment(i*O*D+O, O).transpose(),
                    Vecx.segment(i*O*D+2*O, O).transpose();
        // 以下各种姿势都可以赋值
        // Eigen::Matrix<double, 1, 8> Row;
        // Row = Vecx.segment(i*O*D, O).transpose();
        // coeffMat.row(0) = Row;
        // Row = Vecx.segment(i*O*D+O, O).transpose();
        // coeffMat.row(1) = Row;
        // Row = Vecx.segment(i*O*D+2*O, O).transpose();
        // coeffMat.row(2) = Row;
        //// Why? 分块矩阵赋值就是不行呢 好吧它可以 转置了就行
        // coeffMat.block(0,0,1,O) = Vecx.segment(i*O*D, O).transpose();
        // coeffMat.block(1,0,1,O) = Vecx.segment(i*O*D+O, O).transpose();
        // coeffMat.block(2,0,1,O) = Vecx.segment(i*O*D+2*O, O).transpose();

        // std::cout << coeffMat.block(0,0,1,O) << std::endl;
        // std::cout << coeffMat.block(1,0,1,O) << std::endl;
        // std::cout << coeffMat.block(2,0,1,O) << std::endl;
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
    Eigen::MatrixXd Ewq = Eigen::MatrixXd::Identity(O,O);
    Ewq *= wq;
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
        Qn *=  Ewq;
        // Qn.diagonal() *= wq; 这样只是缩放了对角线的元素
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
            beq_wp((E+1)*KDi,0) = Waypoints(i,k+1);
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

inline void NonTrajOpt::updateAeqbeqDiv() {
    // set zero
    MatAeqx.setZero();  MatAeqy.setZero();  MatAeqq.setZero();
    Vecbeqx.setZero();  Vecbeqy.setZero();  Vecbeqq.setZero();
    // StartState Constraints &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
    MatAeqx.block(0, 0, E, O) = getCoeffCons(0.0);
    MatAeqy.block(0, 0, E, O) = getCoeffCons(0.0);
    MatAeqq.block(0, 0, E, O) = getCoeffCons(0.0);
    Vecbeqx.segment(0, E) = StartStates.row(0).transpose();
    Vecbeqy.segment(0, E) = StartStates.row(1).transpose();
    Vecbeqq.segment(0, E) = StartStates.row(2).transpose();
    // Waypoints and Equality Constraints &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
    Eigen::MatrixXd Matn = -1*getCoeffCons(0.0);
    for (int k = 0; k < N-1; k++) {
        double ti = initT(k);
        Eigen::MatrixXd coeff = getCoeffCons(ti,O,1);
        Eigen::MatrixXd Matp = getCoeffCons(ti);
        int row_index = E + k * (E + 1);
        int col_index = k * O;
        MatAeqx.block(row_index, col_index, 1, O) = coeff;
        MatAeqx.block(row_index + 1, col_index, E, O) = Matp;
        MatAeqx.block(row_index + 1, col_index + O, E, O) = Matn;
        Vecbeqx(row_index,0) = Waypoints(0,k+1); // x mid point as pose

        MatAeqy.block(row_index, col_index, 1, O) = coeff;
        MatAeqy.block(row_index + 1, col_index, E, O) = Matp;
        MatAeqy.block(row_index + 1, col_index + O, E, O) = Matn;
        Vecbeqy(row_index,0) = Waypoints(1,k+1); // y mid point as pose
        
        MatAeqq.block(row_index, col_index, 1, O) = coeff;
        MatAeqq.block(row_index + 1, col_index, E, O) = Matp;
        MatAeqq.block(row_index + 1, col_index + O, E, O) = Matn;
        Vecbeqq(row_index,0) = Waypoints(2,k+1); // q mid point as pose
    }
    // EndState Constraints &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
    double te = initT(N-1); // last element
    int row_index = E + (N-1) * (E + 1);
    int col_index = (N-1) * O;
    MatAeqx.block(row_index, col_index, E, O) = getCoeffCons(te);
    Vecbeqx.segment(row_index, E) = EndStates.row(0).transpose();
    MatAeqy.block(row_index, col_index, E, O) = getCoeffCons(te);
    Vecbeqy.segment(row_index, E) = EndStates.row(1).transpose();
    MatAeqq.block(row_index, col_index, E, O) = getCoeffCons(te);
    Vecbeqq.segment(row_index, E) = EndStates.row(2).transpose();
}

inline void NonTrajOpt::updateMatQDiv() {
    MatkQ.setZero();
    for (int n = 0; n < N; n++) {
        Eigen::MatrixXd Qn = Eigen::MatrixXd::Zero(O, O);
        double ts = initT(n);
        for (int i = C; i < O;i++){
            for (int j = C; j < O;j++){
                Qn(i,j) = factorial(i)/factorial(i-C)*factorial(j)/factorial(j-C)*pow(ts, i+j-2*C+1)/(i+j-2*C+1);
            }
        }
        MatkQ.block(n*O, n*O, O, O) = Qn;
    }
}

// ##############################################################################################################
// Objective Functions 
// ##############################################################################################################
double NonTrajOpt::NLoptobjection(const std::vector<double>& optx, std::vector<double>& grad,void* func_data) {
    NonTrajOpt* OPTptr = reinterpret_cast<NonTrajOpt*>(func_data);
    double Cost;

    OPTptr->iter_num++;

    std::cout << BLUE << "NLopt iter_num: " << RESET << "["<<OPTptr->iter_num <<"] ";//<< std::endl;
    // std::cout << "opt coefs: " ;
    // for (int i = 0; i < OPTptr->OptDof; i++) {
    //     std::cout << optx[i] << " ";
    // }
    // std::cout << std::endl;
    if (OPTptr->TIME_OPTIMIZATION) {
        double opt_time = 0;
        std::cout << "opt times: " ;
        for (int i = 0; i < OPTptr->N; i++) {
            opt_time = optx[OPTptr->OptDof+i];
            // std::cout << "exp(" << opt_time << ")=" << std::exp(opt_time) <<" ";
            std::cout << std::exp(opt_time) <<" ";
        }
        std::cout << std::endl;
    }

    grad.resize(OPTptr->OptNum);
    std::fill(grad.begin(), grad.end(), 0.0);
    Eigen::VectorXd _optx = Eigen::Map<const Eigen::VectorXd>(optx.data(), optx.size());
    Eigen::VectorXd _grad = Eigen::Map<Eigen::VectorXd>(grad.data(), grad.size());

    // std::fill(grad, grad + n, 0.0);
    // Eigen::VectorXd _optx = Eigen::Map<const Eigen::VectorXd>(optx, n);
    // Eigen::VectorXd _grad = Eigen::Map<Eigen::VectorXd>(grad, n);

    // 更新优化变量
    OPTptr->updateOptVars(_optx);
    // 更新优化变量对应的矩阵 并求解 Ax=b
    OPTptr->updateOptAxb();
    // 将求解后的系数放入 Traj 中
    OPTptr->updateTraj();
    // 计算 cost

    if (OPTptr->SMO_SWITCH) {
        double smoCost = 0;
        Eigen::VectorXd gradc = Eigen::VectorXd::Zero(OPTptr->MatDim);
        Eigen::VectorXd gradt = Eigen::VectorXd::Zero(OPTptr->N);
        OPTptr->calcuSmoCost(smoCost,gradc,gradt);
        Cost += smoCost * OPTptr->lambda_smo;
        OPTptr->getReduceOrder(gradc);
        gradc *= OPTptr->lambda_smo;
        gradt *= OPTptr->lambda_smo;
        _grad.head(OPTptr->OptDof) += gradc;
        if (OPTptr->TIME_OPTIMIZATION)
            _grad.tail(OPTptr->N) += gradt;
        
        std::cout << " smoCost: " << smoCost * OPTptr->lambda_smo;

    }
    if (OPTptr->OBS_SWITCH) {
        double obsCost = 0;
        Eigen::VectorXd gradc = Eigen::VectorXd::Zero(OPTptr->MatDim);
        Eigen::VectorXd gradt = Eigen::VectorXd::Zero(OPTptr->N);
        OPTptr->calcuObsCost(obsCost,gradc,gradt);
        Cost += obsCost * OPTptr->lambda_obs;
        OPTptr->getReduceOrder(gradc);
        gradc *= OPTptr->lambda_obs;
        gradt *= OPTptr->lambda_obs;
        _grad.head(OPTptr->OptDof) += gradc;
        if (OPTptr->TIME_OPTIMIZATION)
            _grad.tail(OPTptr->N) += gradt;
        
        std::cout << " obsCost: " << obsCost * OPTptr->lambda_obs;

    }
    if (OPTptr->DYN_SWITCH) {
        double dynCost = 0;
        Eigen::VectorXd gradc = Eigen::VectorXd::Zero(OPTptr->MatDim);
        Eigen::VectorXd gradt = Eigen::VectorXd::Zero(OPTptr->N);
        OPTptr->calcuDynCost(dynCost,gradc,gradt);
        Cost += dynCost * OPTptr->lambda_dyn;
        OPTptr->getReduceOrder(gradc);
        gradc *= OPTptr->lambda_dyn;
        gradt *= OPTptr->lambda_dyn;
        _grad.head(OPTptr->OptDof) += gradc;
        if (OPTptr->TIME_OPTIMIZATION)
            _grad.tail(OPTptr->N) += gradt;

        std::cout << " dynCost: " << dynCost * OPTptr->lambda_dyn;

    }
    if (OPTptr->TIM_SWITCH) {
        double timCost = 0;
        Eigen::VectorXd gradc = Eigen::VectorXd::Zero(OPTptr->MatDim);
        Eigen::VectorXd gradt = Eigen::VectorXd::Zero(OPTptr->N);
        OPTptr->calcuTimCost(timCost,gradc,gradt);
        Cost += timCost * OPTptr->lambda_tim;
        OPTptr->getReduceOrder(gradc);
        gradc *= OPTptr->lambda_tim;
        gradt *= OPTptr->lambda_tim;
        _grad.head(OPTptr->OptDof) += gradc;
        if (OPTptr->TIME_OPTIMIZATION)
            _grad.tail(OPTptr->N) += gradt;

        std::cout << " timCost: " << timCost * OPTptr->lambda_tim;
        
    }
    if (OPTptr->OVA_SWITCH) {
        double ovaCost = 0;
        Eigen::VectorXd gradc = Eigen::VectorXd::Zero(OPTptr->MatDim);
        Eigen::VectorXd gradt = Eigen::VectorXd::Zero(OPTptr->N);
        OPTptr->calcuOvaCost(ovaCost,gradc,gradt);
        Cost += ovaCost * OPTptr->lambda_ova;
        OPTptr->getReduceOrder(gradc);
        gradc *= OPTptr->lambda_ova;
        gradt *= OPTptr->lambda_ova;
        _grad.head(OPTptr->OptDof) += gradc;
        if (OPTptr->TIME_OPTIMIZATION)
            _grad.tail(OPTptr->N) += gradt;

        std::cout << " ovaCost: " << ovaCost * OPTptr->lambda_ova;
    }
    std::cout << " Cost: " << Cost << std::endl;
    // 将 Eigen::VectorXd _grad 赋值给 std::vector<double> grad
    grad.assign(_grad.data(), _grad.data() + _grad.size());

    // std::memcpy(grad, _grad.data(), OPTptr->N * sizeof(double));

    return Cost;
}

double NonTrajOpt::LBFGSobjection(void* ptrObj,const double* optx,double* grad,const int n) {
    NonTrajOpt* OPTptr = reinterpret_cast<NonTrajOpt*>(ptrObj);
    double Cost;

    OPTptr->iter_num++;
    std::cout << BLUE << "NLopt iter_num: " << RESET <<OPTptr->iter_num << std::endl;

    std::fill(grad, grad + n, 0.0);
    Eigen::VectorXd _optx = Eigen::Map<const Eigen::VectorXd>(optx, n);
    Eigen::VectorXd _grad = Eigen::Map<Eigen::VectorXd>(grad, n);

    // std::cout << "opt coefs: " ;
    // for (int i = 0; i < OPTptr->OptDof; i++) {
    //     std::cout << optx[i] << " ";
    // }
    // std::cout << std::endl;
    // if (OPTptr->TIME_OPTIMIZATION) {
    //     double opt_time = 0;
    //     std::cout << "opt times: " ;
    //     for (int i = 0; i < OPTptr->N; i++) {
    //         opt_time = optx[OPTptr->OptDof+i];
    //         std::cout << "exp(" << optx[OPTptr->OptDof+i] << ")=" << std::exp(opt_time) <<" ";
    //     }
    //     std::cout << std::endl;
    // }

    // 更新优化变量
    OPTptr->updateOptVars(_optx);
    // 更新优化变量对应的矩阵 并求解 Ax=b
    OPTptr->updateOptAxb();
    // 将求解后的系数放入 Traj 中
    OPTptr->updateTraj();
    // 计算 cost

    if (OPTptr->SMO_SWITCH) {
        double smoCost = 0;
        Eigen::VectorXd gradc = Eigen::VectorXd::Zero(OPTptr->MatDim);
        Eigen::VectorXd gradt = Eigen::VectorXd::Zero(OPTptr->N);
        OPTptr->calcuSmoCost(smoCost,gradc,gradt);
        Cost += smoCost * OPTptr->lambda_smo;
        OPTptr->getReduceOrder(gradc);
        gradc *= OPTptr->lambda_smo;
        gradt *= OPTptr->lambda_smo;
        _grad.head(OPTptr->OptDof) += gradc;
        if (OPTptr->TIME_OPTIMIZATION)
            if (OPTptr->TIME_OPTIMIZATION)
            _grad.tail(OPTptr->N) += gradt;
    }
    if (OPTptr->OBS_SWITCH) {
        double obsCost = 0;
        Eigen::VectorXd gradc = Eigen::VectorXd::Zero(OPTptr->MatDim);
        Eigen::VectorXd gradt = Eigen::VectorXd::Zero(OPTptr->N);
        OPTptr->calcuObsCost(obsCost,gradc,gradt);
        Cost += obsCost * OPTptr->lambda_obs;
        OPTptr->getReduceOrder(gradc);
        gradc *= OPTptr->lambda_obs;
        gradt *= OPTptr->lambda_obs;
        _grad.head(OPTptr->OptDof) += gradc;
        if (OPTptr->TIME_OPTIMIZATION)
            _grad.tail(OPTptr->N) += gradt;
    }
    if (OPTptr->DYN_SWITCH) {
        double dynCost = 0;
        Eigen::VectorXd gradc = Eigen::VectorXd::Zero(OPTptr->MatDim);
        Eigen::VectorXd gradt = Eigen::VectorXd::Zero(OPTptr->N);
        OPTptr->calcuDynCost(dynCost,gradc,gradt);
        Cost += dynCost * OPTptr->lambda_dyn;
        OPTptr->getReduceOrder(gradc);
        gradc *= OPTptr->lambda_dyn;
        gradt *= OPTptr->lambda_dyn;
        _grad.head(OPTptr->OptDof) += gradc;
        if (OPTptr->TIME_OPTIMIZATION)
            _grad.tail(OPTptr->N) += gradt;
    }
    if (OPTptr->TIM_SWITCH) {
        double timCost = 0;
        Eigen::VectorXd gradc = Eigen::VectorXd::Zero(OPTptr->MatDim);
        Eigen::VectorXd gradt = Eigen::VectorXd::Zero(OPTptr->N);
        OPTptr->calcuTimCost(timCost,gradc,gradt);
        Cost += timCost * OPTptr->lambda_tim;
        OPTptr->getReduceOrder(gradc);
        gradc *= OPTptr->lambda_tim;
        gradt *= OPTptr->lambda_tim;
        _grad.head(OPTptr->OptDof) += gradc;
        if (OPTptr->TIME_OPTIMIZATION)
            _grad.tail(OPTptr->N) += gradt;
    }
    if (OPTptr->OVA_SWITCH) {
        double Ovacost = 0;
        Eigen::VectorXd gradc = Eigen::VectorXd::Zero(OPTptr->MatDim);
        Eigen::VectorXd gradt = Eigen::VectorXd::Zero(OPTptr->N);
        OPTptr->calcuOvaCost(Ovacost,gradc,gradt);
        Cost += Ovacost * OPTptr->lambda_ova;
        OPTptr->getReduceOrder(gradc);
        gradc *= OPTptr->lambda_ova;
        gradt *= OPTptr->lambda_ova;
        _grad.head(OPTptr->OptDof) += gradc;
        if (OPTptr->TIME_OPTIMIZATION)
            _grad.tail(OPTptr->N) += gradt;
    }
    // 更新优化变量 将 Eigen::VectorXd _grad 赋值给 double* grad
    std::cout << " Cost: " << Cost << std::endl;
    std::memcpy(grad, _grad.data(), OPTptr->N * sizeof(double));
    return Cost;
}
// ##############################################################################################################
// Optimization Solvers #########################################################################################
// ##############################################################################################################
// OSQP Function //TODO: add inequality constraints for waypoints max/min states
bool NonTrajOpt::OSQPSolveDiv() {
    //// OSQP prepare
    bool flag = false;
    int succeed = 0;
    Eigen::SparseMatrix<double> Hessian_sparse = MatkQ.sparseView();
    Eigen::VectorXd gradient = Eigen::VectorXd::Zero(MatDimDiv);
    Eigen::VectorXd QPoptx = Eigen::VectorXd::Zero(MatDimDiv);
    Eigen::SparseMatrix<double> MatAeq_sparse ;
    Eigen::VectorXd lowerBound ;
    Eigen::VectorXd upperBound ;

    //// OSQP for x
    flag = false;
    MatAeq_sparse = MatAeqx.sparseView();
    lowerBound = Vecbeqx;
    upperBound = Vecbeqx;
    OsqpEigen::Solver x_solver;
    x_solver.settings()->setWarmStart(true);
    x_solver.settings()->setVerbosity(false);
    x_solver.data()->setNumberOfVariables(MatDimDiv);
    x_solver.data()->setNumberOfConstraints(EquDimDiv);
    if(!x_solver.data()->setHessianMatrix(Hessian_sparse)) return flag;
    if(!x_solver.data()->setGradient(gradient)) return flag;
    if(!x_solver.data()->setLinearConstraintsMatrix(MatAeq_sparse)) return flag;
    if(!x_solver.data()->setLowerBound(lowerBound)) return flag;
    if(!x_solver.data()->setUpperBound(upperBound)) return flag;
    if(!x_solver.initSolver()) return flag;
    if(x_solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError) return flag;
    else {
        flag = true;
        QPoptx = x_solver.getSolution();
        QPxDiv = QPoptx;
        succeed ++;
        // std::cout << "[\033[32mOptimization\033[0m]: OSQP for [x] succeed happy!!!" << std::endl;
    }
    
    //// OSQP for y
    flag = false;
    MatAeq_sparse = MatAeqy.sparseView();
    lowerBound = Vecbeqy;
    upperBound = Vecbeqy;
    OsqpEigen::Solver y_solver;
    y_solver.settings()->setWarmStart(true);
    y_solver.settings()->setVerbosity(false);
    y_solver.data()->setNumberOfVariables(MatDimDiv);
    y_solver.data()->setNumberOfConstraints(EquDimDiv);
    if(!y_solver.data()->setHessianMatrix(Hessian_sparse)) return flag;
    if(!y_solver.data()->setGradient(gradient)) return flag;
    if(!y_solver.data()->setLinearConstraintsMatrix(MatAeq_sparse)) return flag;
    if(!y_solver.data()->setLowerBound(lowerBound)) return flag;
    if(!y_solver.data()->setUpperBound(upperBound)) return flag;
    if(!y_solver.initSolver()) return flag;
    if(y_solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError) return flag;
    else {
        flag = true;
        QPoptx = y_solver.getSolution();
        QPyDiv = QPoptx;
        succeed ++;
        // std::cout << "[\033[32mOptimization\033[0m]: OSQP for [y] succeed happy!!!" << std::endl;
    }

    //// OSQP for q
    flag = false;
    MatAeq_sparse = MatAeqq.sparseView();
    lowerBound = Vecbeqq;
    upperBound = Vecbeqq;
    OsqpEigen::Solver q_solver;
    q_solver.settings()->setWarmStart(true);
    q_solver.settings()->setVerbosity(false);
    q_solver.data()->setNumberOfVariables(MatDimDiv);
    q_solver.data()->setNumberOfConstraints(EquDimDiv);
    if(!q_solver.data()->setHessianMatrix(Hessian_sparse)) return flag;
    if(!q_solver.data()->setGradient(gradient)) return flag;
    if(!q_solver.data()->setLinearConstraintsMatrix(MatAeq_sparse)) return flag;
    if(!q_solver.data()->setLowerBound(lowerBound)) return flag;
    if(!q_solver.data()->setUpperBound(upperBound)) return flag;
    if(!q_solver.initSolver()) return flag;
    if(q_solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError) return flag;
    else {
        flag = true;
        QPoptx = q_solver.getSolution();
        QPqDiv = QPoptx;
        succeed ++;
        // std::cout << "[\033[32mOptimization\033[0m]: OSQP for [q] succeed happy!!!" << std::endl;
    }

    if (succeed == 3) {
        flag = true;
        std::cout << "[\033[32mOptimization\033[0m]: OSQP for [x,y,q] succeed happy!!!" << std::endl;
        for (int idx = 0; idx < N; idx ++) {
            Vecx.segment(idx * O * D, O) = QPxDiv.segment(idx*O, O);
            Vecx.segment(idx * O * D + O, O) = QPyDiv.segment(idx*O, O);
            Vecx.segment(idx * O * D + 2*O, O) = QPqDiv.segment(idx*O, O);
        }
    }
    else {
        flag = false;
        std::cout << "[\033[31mOptimization\033[0m]: OSQP for [x,y,q] failed!!!" << std::endl;
    }

    return flag;
}


bool NonTrajOpt::OSQPSolve() {
    bool flag = false;

    // updateAeqbeq();    // update MatA, Vecb by time initT  
    // updateMatQ();      // update MatQ by time initT

    Eigen::SparseMatrix<double> Hessian_sparse = MatQ.sparseView();

    Eigen::SparseMatrix<double> MatAeq_sparse = MatAeq.sparseView();

    //// 起码打印出来看看 这矩阵挺正常的呀 BUT 为什么 non-convex 呢
    // std::cout << "MatAeq_sparse : \n" << MatAeq_sparse << std::endl;
    // std::cout << "Hessian_sparse: \n" << Hessian_sparse << std::endl;

    Eigen::VectorXd gradient = Eigen::VectorXd::Zero(MatDim);
    Eigen::VectorXd lowerBound = Vecbeq;
    Eigen::VectorXd upperBound = Vecbeq;
    Eigen::VectorXd QPoptx = Eigen::VectorXd::Zero(MatDim);

    //// 这里就没必要设置 上下界的微小偏移了
    // lowerBound -= Eigen::VectorXd::Constant(lowerBound.size(), -0.01);
    // upperBound += Eigen::VectorXd::Constant(upperBound.size(), 0.01);


    // instantiate the solver
    OsqpEigen::Solver solver;

    // settings
    solver.settings()->setWarmStart(true);

    // set the initial data of the QP solver
    solver.data()->setNumberOfVariables(MatDim);
    solver.data()->setNumberOfConstraints(EquDim);
    if(!solver.data()->setHessianMatrix(Hessian_sparse)) return flag;
    if(!solver.data()->setGradient(gradient)) return flag;
    if(!solver.data()->setLinearConstraintsMatrix(MatAeq_sparse)) return flag;
    if(!solver.data()->setLowerBound(lowerBound)) return flag;
    if(!solver.data()->setUpperBound(upperBound)) return flag;

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
    if(!solver.initSolver()) return flag;

    if(solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError) return flag;
    else {
        flag = true;
        QPoptx = solver.getSolution();
        std::cout << "[\033[32mOptimization\033[0m]: OSQP succeed happy!!!" << std::endl;
        // std::cout << "OSQP Solver result: " << std::endl;
        // std::cout << QPoptx << std::endl;
    }

    OSQP_optx = QPoptx;
    Vecx = QPoptx;

    return flag;
}

// NLopt Function
bool NonTrajOpt::NLoptSolve() {
    bool flag = false;
    nlopt::opt opt(nlopt::algorithm::LD_TNEWTON_PRECOND_RESTART, OptNum);
    // // ❗❗❗ NLoptobjection 必须返回 static double 类型的值 且该类型只能声明时候有，定义函数的时候就不能写 static 了
    opt.set_min_objective(NonTrajOpt::NLoptobjection, this);

    std::vector<double> ub(OptNum);
    std::vector<double> lb(OptNum);
    
    Eigen::VectorXd c_upperb = Eigen::VectorXd::Constant(OptDof,COEFFS_UPPER_BOUND);
    Eigen::VectorXd t_upperb = Eigen::VectorXd::Constant(N,TIME_TAU_UPPER_BOUND);
    Eigen::VectorXd c_lowerb = Eigen::VectorXd::Constant(OptDof,COEFFS_LOWER_BOUND);
    Eigen::VectorXd t_lowerb = Eigen::VectorXd::Constant(N,TIME_TAU_LOWER_BOUND);


    Eigen::VectorXd initc = Eigen::VectorXd::Constant(OptDof,INIT_COEFFS);
    Eigen::VectorXd initt = Eigen::VectorXd::Constant(N,INIT_TIME_TAU);


    opt.set_ftol_rel(nlopt_ftol_rel_);
    opt.set_xtol_rel(nlopt_xtol_rel_);
    opt.set_maxeval(nlopt_max_iteration_num_);
    opt.set_maxtime(nlopt_max_iteration_time_);


    std::copy(c_lowerb.data(), c_lowerb.data() + OptDof, lb.begin()); // 将 coeffs lower_bound 的值赋给 lb 的前 OptDof 个元素
    std:copy(c_upperb.data(), c_upperb.data() + OptDof, ub.begin()); // 将 coeffs upper_bound 的值赋给 ub 的前 OptDof 个元素
    if (TIME_OPTIMIZATION) {
        std::copy(t_lowerb.data(), t_lowerb.data() + N, lb.end() - N); // 将 time tau lower_bound 的值赋给 lb 的末尾 N 个元素
        std::copy(t_upperb.data(), t_upperb.data() + N, ub.end() - N); // 将 time tau upper_bound 的值赋给 ub 的末尾 N 个元素
    }

    if (BOUND_OPTIMIZATION) {
        opt.set_lower_bounds(lb);
        opt.set_upper_bounds(ub);
    }

    double minf;
    std::vector<double> NLoptx(OptNum); // NLoptx = [Optc; Optt]

    if (INIT_OPT_VALUE) {
        std::copy(Optx.data(), Optx.data() + OptDof, NLoptx.begin()); // 将Optx的值赋给NLoptx的前OptDof个元素
        if (TIME_OPTIMIZATION) {
            std::copy(Optt.data(), Optt.data() + N, NLoptx.end() - N); // 将Timex的值赋给NLoptx的末尾Tnum个元素
        }
    }
    else {
        std::copy(initc.data(), initc.data() + OptDof, NLoptx.begin()); // 将Optx的值赋给NLoptx的前OptDof个元素
        if (TIME_OPTIMIZATION){
            std::copy(initt.data(), initt.data() + N, NLoptx.end() - N); // 将Timex的值赋给NLoptx的末尾Tnum个元素
        }
    }

    auto nlopt_time_start = std::chrono::high_resolution_clock::now();
    try {

        nlopt::result result = opt.optimize(NLoptx, minf);
        std::cout << "[\033[34mOptimization\033[0m]: nlopt solver" << std::endl;
        std::cout << "NLopt result: " << result << std::endl;

        //// Time clock &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
        auto nlopt_time_end = std::chrono::high_resolution_clock::now();
        // std::chrono::microseconds获取微秒数 std::chrono::milliseconds获取毫秒数，或者使用std::chrono::seconds获取秒数
        auto nlopt_time_duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(nlopt_time_end - nlopt_time_start);
        auto nlopt_time_duration_us = std::chrono::duration_cast<std::chrono::microseconds>(nlopt_time_end - nlopt_time_start);
        auto nlopt_time_duration_s = std::chrono::duration_cast<std::chrono::seconds>(nlopt_time_end - nlopt_time_start);
        std::cout << "NLopt time: " << nlopt_time_duration_us.count() << " us" << std::endl;
        std::cout << "NLopt time: " << nlopt_time_duration_ms.count() << " ms" << std::endl;
        std::cout << "NLopt time: " << nlopt_time_duration_s.count() << " s" << std::endl;
        std::cout << "NLopt minf: " << minf << std::endl;
        //// Debug result flag 
        switch (result) {
        case nlopt::FAILURE:
            std::cout << "nlopt failed!" << std::endl;
            break;
        case nlopt::INVALID_ARGS:
            std::cout << "nlopt invalid args!" << std::endl;
            break;
        case nlopt::OUT_OF_MEMORY:
            std::cout << "nlopt out of memory!" << std::endl;
            break;
        case nlopt::ROUNDOFF_LIMITED:
            std::cout << "nlopt roundoff limited!" << std::endl;
            break;
        case nlopt::FORCED_STOP:
            std::cout << "nlopt forced stop!" << std::endl;
            break;
        case nlopt::SUCCESS:
            std::cout << "nlopt success!" << std::endl;
            break;
        case nlopt::STOPVAL_REACHED:
            std::cout << "nlopt stopval reached!" << std::endl;
            break;
        case nlopt::FTOL_REACHED:
            std::cout << "nlopt ftol reached!" << std::endl;
            break;
        case nlopt::XTOL_REACHED:
            std::cout << "nlopt xtol reached!" << std::endl;
            break;
        case nlopt::MAXEVAL_REACHED:
            std::cout << "nlopt maxeval reached!" << std::endl;
            break;
        case nlopt::MAXTIME_REACHED:
            std::cout << "nlopt maxtime reached!" << std::endl;
            break;
        default:
            std::cout << "nlopt unknown result!" << std::endl;
            break;
        }

        if (result > 0)
            flag = true;
    }
    catch(std::exception& e) {
        std::cout << "[\033[31mOptimization\033[0m]: nlopt exception" << std::endl;
        std::cout << "NLopt failed: " << RED << e.what() << RESET << std::endl;

        //// Time clock &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
        auto nlopt_time_end = std::chrono::high_resolution_clock::now();
        // std::chrono::microseconds获取微秒数 std::chrono::milliseconds 获取毫秒数，或者使用 std::chrono::seconds 获取秒数
        auto nlopt_time_duration = std::chrono::duration_cast<std::chrono::milliseconds>(nlopt_time_end - nlopt_time_start);
        std::cout << "NLopt time: " << nlopt_time_duration.count() << " ms" << std::endl;
    }

    Non_optx = Eigen::Map<const Eigen::VectorXd>(NLoptx.data(), NLoptx.size());
    
    // // 更新优化变量
    // updateOptVars(Non_optx);
    // // 更新优化变量对应的矩阵 并求解 Ax=b
    // updateOptAxb();
    // // 将求解后的系数放入 Traj 中
    // updateTraj();
    return flag;
}

// LBFGS Function
bool NonTrajOpt::LBFGSSolve() {
    bool flag = false;
    
    //// lbfgs 的这些参数都是什么作用呢
    lbfgs::lbfgs_parameter_t lbfgs_params;
    lbfgs::lbfgs_load_default_parameters(&lbfgs_params);
    lbfgs_params.mem_size = 32;
    lbfgs_params.g_epsilon = 0.0;
    lbfgs_params.past = 3;
    lbfgs_params.delta = 1e-4;
    lbfgs_params.max_iterations = 0;
    lbfgs_params.max_linesearch = 60; //default 60
    // lbfgs_params.min_step = 1e-16;
    lbfgs_params.xtol = 1e-4;
    lbfgs_params.line_search_type = 0;

    double* optx_;
    optx_ = new double[OptNum]; // optx_ = [Optc; Optt]
    Eigen::VectorXd initc = Eigen::VectorXd::Constant(OptDof,INIT_COEFFS);
    Eigen::VectorXd initt = Eigen::VectorXd::Constant(N,INIT_TIME_TAU);

    if (INIT_OPT_VALUE) {
        std::memcpy(optx_, Optx.data(), OptDof* sizeof(double));
        if (TIME_OPTIMIZATION) {
            std::memcpy(optx_ + OptDof, Optt.data(), N * sizeof(double));
        }
    }
    else {
        std::memcpy(optx_, initc.data(), OptDof * sizeof(double));
        if (TIME_OPTIMIZATION){
            std::memcpy(optx_ + OptDof, initt.data(), N * sizeof(double));
        }
    }

    std::cout << "Optx: ";
    for (int i = 0; i < OptNum; i++) {
        std::cout << Optx[i] << " ";
    }
    std::cout << std::endl;

    std::cout << "optx_: " ;
    for (int i = 0; i < OptNum; i++) {
        std::cout << optx_[i] << " ";
    }
    std::cout << std::endl;

    double minObjective;

    int opt_ret = 0;

    opt_ret = lbfgs::lbfgs_optimize(OptNum, optx_, &minObjective,
                                  &NonTrajOpt::LBFGSobjection, nullptr,
                                  nullptr, this, &lbfgs_params);

    std::cout << "\033[32m>ret: " << opt_ret << "\033[0m  " 
                << lbfgs::lbfgs_strerror(opt_ret)
                << std::endl;
    
    if (opt_ret < 0) {
        delete[] optx_;
        return false;
    }

    Eigen::VectorXd _optxVec = Eigen::Map<const Eigen::VectorXd>(optx_, OptNum);
    updateOptVars(_optxVec);
    updateOptAxb();
    updateTraj();

    delete[] optx_;
    flag = true;
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
void NonTrajOpt::calcuSmoCost(double &Cost,Eigen::VectorXd &gradc, Eigen::VectorXd &gradt) {
    // smoCost = 0.0;
    // Eigen::VectorXd gradc = Eigen::VectorXd::Zero(MatDim);
    // Eigen::VectorXd gradt = Eigen::VectorXd::Zero(N);
    gdsmotc.resize(3*10,N);
    gdsmocostgrad.resize(6,N);
    for (int idx = 0; idx < N; idx++) {
        Eigen::VectorXd xcoef = Vecx.segment(idx*O*D, O);
        double xcost = getSegPolyCost(xcoef, idx);
        double xgradt = getSegPolyGradt(xcoef, idx);
        // std::cout << "xcoef: " << xcoef.transpose() << std::endl;
        ////&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
        gdsmotc(0,idx) = 576.0 * xcoef(4) * xcoef(4);
        gdsmotc(1,idx) = 5760.0 * xcoef(4) * xcoef(5)*T1(idx);
        gdsmotc(2,idx) = 14400.0 * xcoef(5) * xcoef(5)*T2(idx);
        gdsmotc(3,idx) = 17280.0 * xcoef(4) * xcoef(6)*T2(idx);
        gdsmotc(4,idx) = 86400.0 * xcoef(5) * xcoef(6)*T3(idx);
        gdsmotc(5,idx) = 40320.0 * xcoef(4) * xcoef(7)*T3(idx);
        gdsmotc(6,idx) = 129600.0 * xcoef(6) * xcoef(6)*T4(idx);
        gdsmotc(7,idx) = 201600.0 * xcoef(5) * xcoef(7)*T4(idx);
        gdsmotc(8,idx) = 604800.0 * xcoef(6) * xcoef(7)*T5(idx);
        gdsmotc(9,idx) = 705600.0 * xcoef(7) * xcoef(7)*T6(idx);
        
        getSegPolyGradc(xcoef, idx);
        Eigen::VectorXd ycoef = Vecx.segment(idx*O*D+O, O);
        double ycost = getSegPolyCost(ycoef, idx);
        double ygradt = getSegPolyGradt(ycoef, idx);
        // std::cout << "ycoef: " << ycoef.transpose() << std::endl;
        ////&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
        gdsmotc(10,idx) = 576.0 * ycoef(4) * ycoef(4);
        gdsmotc(11,idx) = 5760.0 * ycoef(4) * ycoef(5)*T1(idx);
        gdsmotc(12,idx) = 14400.0 * ycoef(5) * ycoef(5)*T2(idx);
        gdsmotc(13,idx) = 17280.0 * ycoef(4) * ycoef(6)*T2(idx);
        gdsmotc(14,idx) = 86400.0 * ycoef(5) * ycoef(6)*T3(idx);
        gdsmotc(15,idx) = 40320.0 * ycoef(4) * ycoef(7)*T3(idx);
        gdsmotc(16,idx) = 129600.0 * ycoef(6) * ycoef(6)*T4(idx);
        gdsmotc(17,idx) = 201600.0 * ycoef(5) * ycoef(7)*T4(idx);
        gdsmotc(18,idx) = 604800.0 * ycoef(6) * ycoef(7)*T5(idx);
        gdsmotc(19,idx) = 705600.0 * ycoef(7) * ycoef(7)*T6(idx);

        getSegPolyGradc(ycoef, idx);
        Eigen::VectorXd qcoef = Vecx.segment(idx*O*D+2*O, O);
        double qcost = getSegPolyCost(qcoef, idx);
        double qgradt = getSegPolyGradt(qcoef, idx);
        // std::cout << "qcoef: " << qcoef.transpose() << std::endl;
        ////&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
        gdsmotc(20,idx) = 576.0 * qcoef(4) * qcoef(4);
        gdsmotc(21,idx) = 5760.0 * qcoef(4) * qcoef(5)*T1(idx);
        gdsmotc(22,idx) = 14400.0 * qcoef(5) * qcoef(5)*T2(idx);
        gdsmotc(23,idx) = 17280.0 * qcoef(4) * qcoef(6)*T2(idx);
        gdsmotc(24,idx) = 86400.0 * qcoef(5) * qcoef(6)*T3(idx);
        gdsmotc(25,idx) = 40320.0 * qcoef(4) * qcoef(7)*T3(idx);
        gdsmotc(26,idx) = 129600.0 * qcoef(6) * qcoef(6)*T4(idx);
        gdsmotc(27,idx) = 201600.0 * qcoef(5) * qcoef(7)*T4(idx);
        gdsmotc(28,idx) = 604800.0 * qcoef(6) * qcoef(7)*T5(idx);
        gdsmotc(29,idx) = 705600.0 * qcoef(7) * qcoef(7)*T6(idx);

        getSegPolyGradc(qcoef, idx);
        gradc.segment(idx*O*D, O) = xcoef;
        gradc.segment(idx*O*D+O, O) = ycoef;
        gradc.segment(idx*O*D+2*O, O) = qcoef*wq;
        gradt(idx) = (xgradt + ygradt + qgradt*wq)*T1(idx);
        Cost += xcost + ycost + qcost*wq;
        gdsmocostgrad(0,idx) = xgradt;
        gdsmocostgrad(1,idx) = gdsmotc(0,idx) + gdsmotc(1,idx) + gdsmotc(2,idx) + gdsmotc(3,idx) + gdsmotc(4,idx) + 
                               gdsmotc(5,idx) + gdsmotc(6,idx) + gdsmotc(7,idx) + gdsmotc(8,idx) + gdsmotc(9,idx);
        gdsmocostgrad(2,idx) = ygradt;
        gdsmocostgrad(3,idx) = gdsmotc(10,idx) + gdsmotc(11,idx) + gdsmotc(12,idx) + gdsmotc(13,idx) + gdsmotc(14,idx) + 
                               gdsmotc(15,idx) + gdsmotc(16,idx) + gdsmotc(17,idx) + gdsmotc(18,idx) + gdsmotc(19,idx);
        gdsmocostgrad(4,idx) = qgradt;
        gdsmocostgrad(5,idx) = gdsmotc(20,idx) + gdsmotc(21,idx) + gdsmotc(22,idx) + gdsmotc(23,idx) + gdsmotc(24,idx) + 
                               gdsmotc(25,idx) + gdsmotc(26,idx) + gdsmotc(27,idx) + gdsmotc(28,idx) + gdsmotc(29,idx);
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
void NonTrajOpt::calcuObsCost(double &Cost,Eigen::VectorXd &gradc, Eigen::VectorXd &gradt) {
    Eigen::VectorXi DiscreteNums = Traj.getDiscreteNums();
    gdobstc.resize(10,(DiscreteNums.sum()+DiscreteNums.size()));
    gdobstc.setZero();
    gdobstimemat.resize(8,(DiscreteNums.sum()+DiscreteNums.size()));
    gdobstimemat.setZero();
    int gdidx = 0;
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
            Eigen::VectorXd vectime = getTimeVec(Tdt,0);
            if (dist < dist_th) {
                vecCost += 1/std::pow(2*(dist),2) * vel.squaredNorm() * Tdt;
                // Eigen::VectorXd vectime = getTimeVec(Tdt,0);
                xgrad += grad(0)*vectime*(-2*std::pow(dist,3));
                ygrad += + grad(1)*vectime*(-2*std::pow(dist,3));
                // gradc.segment(idx*O*D, O) += 1.0 / (dist * dist); //* edfmap.getGrad(pos).transpose();
            }
            //[xpos ypos dist xgrad ygrad Tdt cost velnorm 1/dist^2 -2*dist^3]
            gdobstc(0,gdidx) = pos(0);
            gdobstc(1,gdidx) = pos(1);
            gdobstc(2,gdidx) = dist;
            gdobstc(3,gdidx) = grad(0);
            gdobstc(4,gdidx) = grad(1);
            gdobstc(5,gdidx) = Tdt;
            gdobstc(6,gdidx) = 1/std::pow(2*(dist),2) * vel.squaredNorm() * Tdt;
            gdobstc(7,gdidx) = vel.squaredNorm();
            gdobstc(8,gdidx) = 1/std::pow(2*(dist),2);
            gdobstc(9,gdidx) = -2*std::pow(dist,3);
            gdobstimemat.col(gdidx) = vectime;
            gdidx++;
        }
        Cost += vecCost;
        gradc.segment(idx*O*D, O) = xgrad;
        gradc.segment(idx*O*D+O, O) = ygrad;
    }
    gradt.setZero();
}


/***************************************************************************************************************
 * @description: calculate dynamic constraints cost
 * @reference: https://gitee.com/hi-zwt/kinodynamicpath/blob/master/PolyOpt/dynamicCost.m
 * @param {double} &Cost
 * @param {VectorXd} gradc
 * @param {VectorXd} gradt
 * @return {*}
 */
void NonTrajOpt::calcuDynCost(double &Cost,Eigen::VectorXd &gradc, Eigen::VectorXd &gradt) {
    Eigen::VectorXi DiscreteNums = Traj.getDiscreteNums();
    gddyntc.resize(14,(DiscreteNums.sum()+DiscreteNums.size()));
    gddyntc.setZero();
    int gdidx = 0;
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
        // std::cout << "velVector size :" << Traj[idx].velVector.rows() << "," << Traj[idx].velVector.cols() << std::endl;
        // std::cout << "DiscreteNum :" << Traj[idx].getDiscreteNum() << std::endl;
        for (int idy = 0; idy <= Traj[idx].getDiscreteNum(); idy++) {
            // std::cout << "velVector.col(idy) :" << Traj[idx].velVector.col(idy) << std::endl;
            // std::cout << "accVector.col(idy) :" << Traj[idx].accVector.col(idy) << std::endl;
            // std::cout << "jerVector.col(idy) :" << Traj[idx].jerVector.col(idy) << std::endl;
            // Eigen::Vector3d vel(1.1,1.1,1.1);
            // Eigen::Vector3d acc(2.2,2.2,2.2);
            // Eigen::Vector3d jer(3.3,3.3,3.3);     
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
            velgradt += 2*deltaVel.dot(acc)*Tdt;
            accgradt += 2*deltaAcc.dot(jer)*Tdt;
            ////Debug: [Tdt deltaVel deltaAcc velCost accCost velgradt accgradt velnorm accnorm jernorm]
            gddyntc(0,gdidx) = Tdt;
            gddyntc(1,gdidx) = deltaVel(0);
            gddyntc(2,gdidx) = deltaVel(1);
            gddyntc(3,gdidx) = deltaVel(2);
            gddyntc(4,gdidx) = deltaAcc(0);
            gddyntc(5,gdidx) = deltaAcc(1);
            gddyntc(6,gdidx) = deltaAcc(2);
            gddyntc(7,gdidx) = deltaVel.squaredNorm();
            gddyntc(8,gdidx) = deltaAcc.squaredNorm();
            gddyntc(9,gdidx) = 2*deltaVel.dot(acc);
            gddyntc(10,gdidx) = 2*deltaAcc.dot(jer);
            gddyntc(11,gdidx) = vel.squaredNorm();
            gddyntc(12,gdidx) = acc.squaredNorm();
            gddyntc(13,gdidx) = jer.squaredNorm();
            gdidx++;
        }
        Cost += velCost + accCost;
        // gradc.segment(idx*O*D, O)       = xvgrad;// + xagrad;
        // gradc.segment(idx*O*D+O, O)     = yvgrad;// + yagrad;
        // gradc.segment(idx*O*D+2*O, O)   = qvgrad;// + qagrad;
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
void NonTrajOpt::calcuTimCost(double &Cost,Eigen::VectorXd &gradc, Eigen::VectorXd &gradt) {
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
void NonTrajOpt::calcuOvaCost(double &Cost,Eigen::VectorXd &gradc, Eigen::VectorXd &gradt) {
    Eigen::VectorXi DiscreteNums = Traj.getDiscreteNums();
    gdovatc.resize(17,(DiscreteNums.sum()+DiscreteNums.size()));
    gdovatc.setZero();
    int gdidx = 0;
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
            // rotation matrix and state transform //[x] 判断选择矩阵的方向
            Rmatrix << cosyaw, sinyaw,
                       -sinyaw, cosyaw;
            Eigen::Vector2d vel_B = Rmatrix * vel.head(2);
            Eigen::Vector2d acc_B = Rmatrix * acc.head(2);
            double vel_B2 = std::pow(vel_B(0),2)/ORIEN_VEL2 + std::pow(vel_B(1),2)/VERDIT_VEL2;
            if (vel_B2 > OVAL_TH) {
                Cost += vel_B2 - OVAL_TH;
                gradt(idx) += (2 * vel_B(1) * (acc_B(1) - cosyaw * vel(0) * vel(2) - sinyaw * vel(1) * vel(2)) / VERDIT_VEL2 +
                               2 * vel_B(0) * (acc_B(0) + cosyaw * vel(1) * vel(2) - sinyaw * vel(0) * vel(2)) / ORIEN_VEL2) * Tdt;
                xgrad += (2 * cosyaw * vel_B(0)/ORIEN_VEL2 - 2 * sinyaw * vel_B(1) / VERDIT_VEL2) * vectime2;
                ygrad += (2 * sinyaw * vel_B(0)/ORIEN_VEL2 + 2 * cosyaw * vel_B(1) / VERDIT_VEL2) * vectime2;
                qgrad += (2 * vel_B(0) * vel_B(1) / ORIEN_VEL2 - 2 * vel_B(1) * vel_B(0) / VERDIT_VEL2) * vectime1;
            }
            ////Debug: [Tdt vel_B(1) vel_B(2) vel_B2 R(1,1) R(1,2) R(2,1) R(2,2) gradt(idx) xgrad ygrad qgrad]
            gdovatc(0,gdidx) = Tdt;
            gdovatc(1,gdidx) = vel_B(0);
            gdovatc(2,gdidx) = vel_B(1);
            gdovatc(3,gdidx) = vel_B2;
            gdovatc(4,gdidx) = Rmatrix(0,0);
            gdovatc(5,gdidx) = Rmatrix(0,1);
            gdovatc(6,gdidx) = Rmatrix(1,0);
            gdovatc(7,gdidx) = Rmatrix(1,1);
            gdovatc(8,gdidx) = (2 * vel_B(1) * (acc_B(1) - cosyaw * vel(0) * vel(2) - sinyaw * vel(1) * vel(2)) / VERDIT_VEL2 +
                               2 * vel_B(0) * (acc_B(0) + cosyaw * vel(1) * vel(2) - sinyaw * vel(0) * vel(2)) / ORIEN_VEL2) * Tdt;
            gdovatc(9,gdidx) = (2 * cosyaw * vel_B(0)/ORIEN_VEL2 - 2 * sinyaw * vel_B(1) / VERDIT_VEL2);
            gdovatc(10,gdidx) = (2 * sinyaw * vel_B(0)/ORIEN_VEL2 + 2 * cosyaw * vel_B(1) / VERDIT_VEL2);
            gdovatc(11,gdidx) = (2 * vel_B(0) * vel_B(1) / ORIEN_VEL2 - 2 * vel_B(1) * vel_B(0) / VERDIT_VEL2);
            gdovatc(12,gdidx) = acc_B(0);
            gdovatc(13,gdidx) = acc_B(1);
            gdovatc(14,gdidx) = acc_B.squaredNorm();
            gdovatc(15,gdidx) = acc_B(0)*acc_B(0) + acc_B(1)*acc_B(1);
            gdovatc(16,gdidx) = yaw_q;
            gdidx++;
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