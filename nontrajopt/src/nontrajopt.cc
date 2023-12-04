/*
 * @Author: wentao zhang && zwt190315@163.com
 * @Date: 2023-07-24
 * @LastEditTime: 2023-12-04
 * @Description: trajectory optimization
 * @reference: 
 * 
 * Copyright (c) 2023 by wentao zhang, All Rights Reserved. 
 */

#include "nontrajopt/nontrajopt.h"

// #### debug
#define SHOW_CSOT_SWITCH true

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
void NonTrajOpt::setParam(ros::NodeHandle &nh,OptParas &paras) {
    nh.param("nontrajopt/lambda_smo", paras.lambda_smo, 1.0);
    nh.param("nontrajopt/lambda_obs", paras.lambda_obs, 0.0);
    nh.param("nontrajopt/lambda_dyn", paras.lambda_dyn, 0.0);
    nh.param("nontrajopt/lambda_tim", paras.lambda_tim, 0.0);
    nh.param("nontrajopt/lambda_ova", paras.lambda_ova, 0.0);
    nh.param("nontrajopt/SMO_SWITCH", paras.SMO_SWITCH, false);
    nh.param("nontrajopt/OBS_SWITCH", paras.OBS_SWITCH, false);
    nh.param("nontrajopt/DYN_SWITCH", paras.DYN_SWITCH, false);
    nh.param("nontrajopt/TIM_SWITCH", paras.TIM_SWITCH, false);
    nh.param("nontrajopt/OVA_SWITCH", paras.OVA_SWITCH, false);
    nh.param("nontrajopt/QPC", paras.QPC, 4);
    nh.param("nontrajopt/dist_th", paras.dist_th, 0.1);
    nh.param("nontrajopt/discredist", paras.discredist, 0.1);
    nh.param("nontrajopt/pv_max", paras.pv_max, 2.0);
    nh.param("nontrajopt/pa_max", paras.pa_max, 2.0);
    nh.param("nontrajopt/wv_max", paras.wv_max, 2.0);
    nh.param("nontrajopt/wa_max", paras.wa_max, 2.0);
    nh.param("nontrajopt/dyn_rate", paras.dyn_rate, 1.0);
    nh.param("nontrajopt/OVAL_TH", paras.OVAL_TH, 0.8);
    nh.param("nontrajopt/ORIEN_VEL", paras.ORIEN_VEL, 2.4);
    nh.param("nontrajopt/VERDIT_VEL", paras.VERDIT_VEL, 1.2);
    nh.param("nontrajopt/coeff_bound", paras.coeff_bound, 500.0);
    nh.param("nontrajopt/TIME_OPTIMIZATION", paras.TIME_OPTIMIZATION, false);
    nh.param("nontrajopt/REDUCE_ORDER", paras.REDUCE_ORDER, false);
    nh.param("nontrajopt/BOUND_OPTIMIZATION", paras.BOUND_OPTIMIZATION, false);
    nh.param("nontrajopt/INIT_OPT_VALUE", paras.INIT_OPT_VALUE, false);
    nh.param("nontrajopt/nlopt_max_iteration_num", paras.nlopt_max_iteration_num_, 100);
    nh.param("nontrajopt/nlopt_max_iteration_time", paras.nlopt_max_iteration_time_, 0.1);
    nh.param("nontrajopt/nlopt_xtol_rel", paras.nlopt_xtol_rel_,1e-6);
    nh.param("nontrajopt/nlopt_ftol_rel", paras.nlopt_ftol_rel_,1e-6);

    nh.param("nontrajopt/wq", paras.wq, 0.8);
    nh.param("search/vel_factor", paras.ref_vel, 1.0);
    nh.param("search/ome_factor", paras.ref_ome, 1.0);

    nh.param("map/x_size",paras.map_width, 10.0);
    nh.param("map/y_size",paras.map_height, 10.0);
    nh.param("map/resolution",paras.map_resolution, 0.1);
    nh.param("map/mini_dist",paras.mini_dist, 0.01);
}

void NonTrajOpt::initParameter(const OptParas &paras) {
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

    QPC        = paras.QPC; // cost function order of OSQP Problem
    
    wq         = paras.wq; // weight of state.q coefficients 
    dist_th    = paras.dist_th; // distance threshold of obstacle   
    ref_vel    = paras.ref_vel; // reference linear velocity
    ref_ome    = paras.ref_ome; // reference angular velocity
    dyn_rate   = paras.dyn_rate; // dynamic rate
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

    coeff_bound = paras.coeff_bound; // bound of coefficients
    time_bottom = 0.8;

    //&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
    // Optimization Solver Parameters
    nlopt_max_iteration_num_ = paras.nlopt_max_iteration_num_;
    nlopt_max_iteration_time_ = paras.nlopt_max_iteration_time_;
    nlopt_ftol_rel_ = paras.nlopt_ftol_rel_;
    nlopt_xtol_rel_ = paras.nlopt_xtol_rel_;
    setEDFMap(paras.map_width,paras.map_height,paras.map_resolution,paras.mini_dist);

    O = 8; // 7th = 8 coefficients
    D = 3; // [x,y,q]
    E = 4; // [p,v,a,j]
    C = 4; // cost function order
}

void NonTrajOpt::showParam() {

    std::cout << "[\033[34mnontrajopt\033[0m]Coeffs: " << O ;
    std::cout << "; Dimensions: " << D ;
    std::cout << "; Equations: " << E ;
    std::cout << "; Cost: " << C << std::endl;
    std::cout << "[\033[34mnontrajopt\033[0m]QPC: " << QPC << std::endl;


    std::cout << std::boolalpha << "[\033[34mnontrajopt\033[0m]TIME_OPTIMIZATION: " << TIME_OPTIMIZATION << std::endl;
    std::cout << std::boolalpha << "[\033[34mnontrajopt\033[0m]REDUCE_ORDER: " << REDUCE_ORDER << std::endl;
    std::cout << std::boolalpha << "[\033[34mnontrajopt\033[0m]INIT_OPT_VALUE: " << INIT_OPT_VALUE << std::endl;
    std::cout << std::boolalpha << "[\033[34mnontrajopt\033[0m]BOUND_OPTIMIZATION: " << BOUND_OPTIMIZATION << std::endl;
    
    std::cout << std::boolalpha << "[\033[34mnontrajopt\033[0m]SMO_SWITCH: " << SMO_SWITCH << "; lambda_smo : " << lambda_smo << std::endl;
    std::cout << std::boolalpha << "[\033[34mnontrajopt\033[0m]OBS_SWITCH: " << OBS_SWITCH << "; lambda_obs : " << lambda_obs << std::endl;
    std::cout << std::boolalpha << "[\033[34mnontrajopt\033[0m]DYN_SWITCH: " << DYN_SWITCH << "; lambda_dyn : " << lambda_dyn << std::endl;
    std::cout << std::boolalpha << "[\033[34mnontrajopt\033[0m]TIM_SWITCH: " << TIM_SWITCH << "; lambda_tim : " << lambda_tim << std::endl;
    std::cout << std::boolalpha << "[\033[34mnontrajopt\033[0m]OVA_SWITCH: " << OVA_SWITCH << "; lambda_ova : " << lambda_ova << std::endl;

    std::cout << "[\033[34mnontrajopt\033[0m]wq: " << wq << "; dist_th: " << dist_th << "; discredist " << discredist << std::endl;
    std::cout << "[\033[34mnontrajopt\033[0m]ref_vel: " << ref_vel << "; ref_ome: " << ref_ome << std::endl;
    std::cout << "[\033[34mnontrajopt\033[0m]pv_max: " << pv_max << "; pa_max: " << pa_max << std::endl;
    std::cout << "[\033[34mnontrajopt\033[0m]wv_max: " << wv_max << "; wa_max: " << wa_max << std::endl;
    std::cout << "[\033[34mnontrajopt\033[0m]OVAL_TH: " << OVAL_TH << "; ORIEN_VEL " << ORIEN_VEL << "; VERDIT_VEL " << VERDIT_VEL <<std::endl;
    std::cout << "[\033[34mnontrajopt\033[0m]coeff_bound: " << coeff_bound << std::endl;
    std::cout << "[\033[34mnontrajopt\033[0m]MaxIterNum: " << nlopt_max_iteration_num_ ;
    std::cout << "; MaxIterTime: "  << nlopt_max_iteration_time_ << std::endl; //<<  std::setprecision(2)
    std::cout << "[\033[34mnontrajopt\033[0m]nlopt_xtol_rel_: " << nlopt_xtol_rel_ << std::endl;

    std::cout << "[\033[34mnontrajopt\033[0m]map_width: " << edfmap._map_size_x << "; map_height: " << edfmap._map_size_y << std::endl;
    std::cout << "[\033[34mnontrajopt\033[0m]map_origin_x: " << edfmap._map_origin_x << "; _map_origin_y: " << edfmap._map_origin_y << std::endl;
    std::cout << "[\033[34mnontrajopt\033[0m]map_resolution: " << edfmap._map_resolution << "; mini_dist: " << edfmap._mini_dist << std::endl;
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
    EndStates.block(0,0,3,3) =_endStates;
    for (int idx = 0; idx < N + 1 ; idx++) {
        Initwaypoints.col(idx) = _waypoints[idx];
    //// Step 0: yaw 角归一化到 [-pi pi] ##################################################################
        Vecyaw(idx) = angles::normalize_angle(_waypoints[idx](2));
    }
    // std::cout << "waypoint q: " << Initwaypoints.row(2) << std::endl;
    // std::cout << "Raw Vecyaw: " << Vecyaw.transpose() << std::endl;
    //// Step 1: yaw 角的平滑处理 缩小使转向角变化
    Eigen::VectorXd VecyawSmooth = Eigen::VectorXd::Zero(_waypoints.size());
    for (int idx = 1; idx < N ; idx++) {
        VecyawSmooth(idx) = angles::shortest_angular_distance(Vecyaw(idx),Vecyaw(idx+1))/2.0 + Vecyaw(idx);
    }
    VecyawSmooth(0) = Vecyaw(0); VecyawSmooth(N) = Vecyaw(N);
    // std::cout << "Smooth Vecyaw: " << VecyawSmooth.transpose() << std::endl;
    Vecyaw = VecyawSmooth;
    //// Step 2: reference time 参考优化时间计算 ###########################################################
    std::cout << "OSQPopt->qdist: ";
    for (int idx = 0; idx < N; idx++) {
        Eigen::Vector2d xypos_s = Initwaypoints.col(idx).head(2);
        Eigen::Vector2d xypos_e = Initwaypoints.col(idx+1).head(2);
        Eigen::Vector2d xypos = xypos_e - xypos_s;
        double xy_dist = xypos.norm();
        double q_dist = angles::shortest_angular_distance(Vecyaw(idx),Vecyaw(idx+1));
        InitT(idx) = std::max(xy_dist/ref_vel,abs(q_dist/ref_ome));
        std::cout << q_dist << "; ";
        // std::cout << "seg = " << idx << "; xy_dist: " << xy_dist << "; q_dist: " << q_dist 
        //         << "; xy-time: " << xy_dist/ref_vel << "; q-time" << q_dist/ref_ome << std::endl;
    }
    std::cout << std::endl;
    //// Step 2.5 : reference time 在收尾增加处理 ###########################################################
    InitT(0) = InitT(0) * (1.0 + TIME_END_FACTOR * std::max(std::abs(StartStates.block(0,1,2,1).norm()-ref_vel)/ref_vel,
                                                            std::abs(StartStates(2,1)-ref_ome)/ref_ome));
    InitT(N-1) = InitT(N-1) * (1.0 + TIME_END_FACTOR * std::max(std::abs(EndStates.block(0,1,2,1).norm()-ref_vel)/ref_vel,
                                                            std::abs(EndStates(2,1)-ref_ome)/ref_ome));
    //// Step 2.6 : time 的底线处理 最小时间需要 >= time_bottom
    for (int idx = 0; idx < N; idx++) {
        if (InitT(idx) < time_bottom) InitT(idx) = time_bottom;
    }
    //// Step 3: yaw 角的数值连续性处理 用于数值优化 #########################################################
    // std::cout << "shortest distance: " << 0.0 << " ";
    for (int idx = 1; idx <= N; idx++) { ////Note: 这里的第 N 个点会不会有问题? 比原定的点偏离? 应该不会吧?
        VecyawSmooth(idx) = VecyawSmooth(idx-1) + angles::shortest_angular_distance(Vecyaw(idx-1),Vecyaw(idx));
        // std::cout << angles::shortest_angular_distance(Vecyaw(idx-1),Vecyaw(idx)) << " ";
    }
    // std::cout << std::endl;
    VecyawSmooth(0) = Vecyaw(0); 
    
    // std::cout << "continus Vecyaw: " << VecyawSmooth.transpose() << std::endl;

    for (int idx = 0; idx <= N; idx++) {
        Initwaypoints(2,idx) = VecyawSmooth(idx);
    }
    //// Step 4: init waypoints and start/end states for polynomial trajectory optimization
    StartStates.col(0) = Initwaypoints.col(0);
    EndStates.col(0) = Initwaypoints.col(N);
    initWaypoints(Initwaypoints,InitT,StartStates,EndStates);
    return true;
}
    // std::cout << "InitT : " << InitT.transpose() << std::endl;
    //// start and end time &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
    // std::cout << "initT: " << InitT.transpose() << std::endl;
    // Eigen::Vector2d dotvel,dotpos;
    // double dotome;
    // dotpos = _startStates.block(0,0,2,1);
    // dotvel = _startStates.block(0,1,2,1);
    // dotome = _startStates(2,1);
    // std::cout << "start pos: " << dotpos.transpose() << std::endl;
    // std::cout << "start vel: " << dotvel.transpose() << "; dotome: " << dotome << std::endl;
    // std::cout << "dotvel.norm() : " << dotvel.norm() << "; time: " << InitT.head(1) << std::endl;
    // InitT.head(1) = InitT.head(1) * (1.0 + std::max(std::abs(dotvel.norm()-ref_vel)/ref_vel,std::abs(dotome-ref_ome)/ref_ome));
    // dotpos = _endStates.block(0,0,2,1);
    // dotvel = _endStates.block(0,1,2,1);
    // dotome = _endStates(2,1);
    // std::cout << "start pos: " << dotpos.transpose() << std::endl;
    // std::cout << "start vel: " << dotvel.transpose() << "; dotome: " << dotome << std::endl;
    // std::cout << "dotvel.norm() : " << dotvel.norm() << "; time: " << InitT.tail(1) << std::endl;
    // InitT.tail(1) = InitT.tail(1) * (1.0 + std::max(std::abs(dotvel.norm()-ref_vel)/ref_vel,std::abs(dotome-ref_ome)/ref_ome));
    // std::cout << "initT: " << InitT.transpose() << std::endl;
    //// &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&


/*****************************************************************************************************
 * @description: push Waypoints without any modification
 * @reference: 
 * @param {std::vector<Eigen::Vector3d>} _waypoints vector of waypoints pos
 * @param {Eigen::Matrix<double, 3, 3>} _startStates pos;vel;acc
 * @param {Eigen::Matrix<double, 3, 3>} _endStates  pos;vel;acc
 * @return {bool} flag of success
 */
bool NonTrajOpt::pushWaypoints(const std::vector<Eigen::Vector3d> &_waypoints, const std::vector<double> &_initT,
    const Eigen::Matrix<double, 3, 3> &_startStates, const Eigen::Matrix<double, 3, 3> &_endStates) {
    if ( _waypoints.size() - N != 1 || _initT.size() - N != 0) {
        return false;
    }
    //// 注意 waypoints 的数量比 轨迹段 N 多 1 ##############################################################
    Eigen::Matrix3Xd Initwaypoints = Eigen::Matrix3Xd::Zero(3, _waypoints.size());
    Eigen::VectorXd Vecyaw = Eigen::VectorXd::Zero(_waypoints.size());
    Eigen::VectorXd InitT = Eigen::Map<const Eigen::VectorXd>(_initT.data(), _initT.size());
    Eigen::Matrix<double, 3, 4> StartStates = Eigen::Matrix<double, 3, 4>::Zero();
    StartStates.block(0,0,3,3) = _startStates;
    Eigen::Matrix<double, 3, 4> EndStates = Eigen::Matrix<double, 3, 4>::Zero();
    EndStates.block(0,0,3,3) = _endStates;
    for (int idx = 0; idx < N + 1 ; idx++) {
        Initwaypoints.col(idx) = _waypoints[idx];
    //// Step 0: yaw 角归一化到 [-pi pi] ##################################################################
        Vecyaw(idx) = angles::normalize_angle(_waypoints[idx](2));
    }
    Eigen::VectorXd VecyawSmooth = Vecyaw;
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

void NonTrajOpt::setEDFMap( const double map_size_x, const double map_size_y, 
                            const double map_resolution, const double mini_dist) {
    edfmap.setMapParam(map_size_x, map_size_y, map_resolution, mini_dist);
}

bool NonTrajOpt::updateEDFMap(const cv::Mat &img) {
    if (img.empty()) {
        std::cout << "edf map is empty" << std::endl;
        return false;
    }
    edfmap.setMap(img);
    return true;
}
bool NonTrajOpt::updateEDFMap(const cv::Mat* img) {
    if (img->empty()) {
        std::cout << "edf map is empty" << std::endl;
        return false;
    }
    edfmap.setMap(img);
    return true;
}
bool NonTrajOpt::readEDFMap(const std::string &filename,const int kernel_size) {
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

void NonTrajOpt::reset(const int &pieceNum) {
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
    for (int id_seg = 0; id_seg < N-1 ; id_seg++) { // N segments 
        for (int id_dim = 0; id_dim < D ; id_dim++) { // 3 dimensions
            for (int id_ord = E + 1 ; id_ord < O ; id_ord++) { // 5~7 orders
                int id_num = id_seg*O*D + id_dim*O + id_ord + EQ_OFFSET;
                optFlag[id_num] = true;
                dof_num++;
            }
        }
    }
    
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
            for (int id_ord = E + 1 ; id_ord < O ; id_ord++) { // 5~7 orders
                int id_num = id_seg*O*D + id_dim*O + id_ord + EQ_OFFSET;
                Eigen::VectorXd tempRow = Eigen::VectorXd::Zero(MatDim);
                tempRow(id_num) = 1.0;
                TempAeqRE.row(id_num) = tempRow.transpose();
                TempbeqRE(id_num) = Optx(dof_num); // Optc 好像是一样的效果?
                dof_num++;
            }
        }
    }
    // std::cout << "dof num : " << dof_num << std::endl;
    int equ_num = 0;
    for (int idx = 0; idx < MatDim && equ_num < EquDim; idx++) {
        if (!optFlag[idx]) {
            TempAeqRE.row(idx) = TempAeq.row(equ_num);
            TempbeqRE(idx) = Tempbeq(equ_num);
            equ_num++;
        }
    }
    // std::cout << "equ num : " << equ_num << std::endl;
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
    MatABef = TempAeq;
    VecbBef = Tempbeq;
    //// 最小二乘求解线性方程组 &&&&&&&&&&&&&&&&&&&&&&&&&&&&&
    //// 正规阵法 Ax=b; A^{T}Ax = A^{T}b
    Vecx = (TempAeqRE.transpose() * TempAeqRE).ldlt().solve(TempAeqRE.transpose() * TempbeqRE);
    //// M-P inv Maybe the best but slowest ################################
    // Vecx = Eigenpinv(TempAeqRE)*Vecb;
    // Eigen::SparseQR<Eigen::SparseMatrix<double>, Eigen::COLAMDOrdering<int>> solver;
    // 设置求解器的参数（可选）
    // solver.setPivotThreshold(1e-6); // 设置主元选取阈值，默认为1e-6
    
    //// Debug MP-pinv
    // Vecx = TempAeqRE.fullPivLu().solve(TempbeqRE);
    

    // if (rank < MatDim) {
    //     // std::cout << YELLOW << "Write TempAeqRANK.csv" << RESET << std::endl;
    //     // std::cout << "Time Vector raw :" << Optt.transpose() << std::endl;
    //     // std::cout << "Time Vector exp :" << Vect.transpose() << std::endl;
    //     // eigenCSV.WriteMatrix(TempAeqRE,"/home/zwt/Documents/TempAeqRANK.csv");
    //     // eigenCSV.WriteVector(TempbeqRE,"/home/zwt/Documents/TempbeqRANK.csv");
    //     // std::cout << std::endl;
    //     // std::cout << YELLOW << "God damn it! TempAeqRE rand :" << rank <<" < " << MatDim << " is singular!" << RESET << std::endl;
    //     std::cout << YELLOW << "Aeq * Vecx = beq rand :" << rank <<" < " << MatDim << " is singular!" << RESET << std::endl;
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
        coeffMats.push_back(coeffMat);
    }
    std::vector<double> times(Vect.data(), Vect.data() + Vect.size()); // 更新traj使用的时间 Vect
    // std::vector<double> times(initT.data(), initT.data() + initT.size());
    std::vector<int> discs(discNums.data(), discNums.data() + discNums.size());
    // std::cout << "times size: " << times.size() << "; discs size: " << discNums.size() << "; coeffMats: " << coeffMats.size() << std::endl;
    // std::cout << "times   : " << Vect.transpose() << std::endl;
    // std::cout << "discNums: " << discNums.transpose() << std::endl;
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
        for (int i = QPC; i < O;i++){
            for (int j = QPC; j < O;j++){
                Qn(i,j) = factorial(i)/factorial(i-QPC)*factorial(j)/factorial(j-QPC)*pow(ts, i+j-2*QPC+1)/(i+j-2*QPC+1);
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
        for (int i = QPC; i < O;i++){
            for (int j = QPC; j < O;j++){
                Qn(i,j) = factorial(i)/factorial(i-QPC)*factorial(j)/factorial(j-QPC)*pow(ts, i+j-2*QPC+1)/(i+j-2*QPC+1);
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
    double Cost = 0.0;

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
            // std::cout << "exp(" << opt_time << ")=" << std::exp(opt_time) <<"; ";
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
        
        if (SHOW_CSOT_SWITCH) std::cout << " smoCost: " << smoCost * OPTptr->lambda_smo;

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
        
        if (SHOW_CSOT_SWITCH) std::cout << " obsCost: " << obsCost * OPTptr->lambda_obs;

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

        if (SHOW_CSOT_SWITCH) std::cout << " dynCost: " << dynCost * OPTptr->lambda_dyn;

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

        if (SHOW_CSOT_SWITCH) std::cout << " timCost: " << timCost * OPTptr->lambda_tim;
        
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

        if (SHOW_CSOT_SWITCH) std::cout << " ovaCost: " << ovaCost * OPTptr->lambda_ova;
    }

    if (SHOW_CSOT_SWITCH) std::cout << " Cost: " << Cost << std::endl;
    // 将 Eigen::VectorXd _grad 赋值给 std::vector<double> grad
    grad.assign(_grad.data(), _grad.data() + _grad.size());

    // std::memcpy(grad, _grad.data(), OPTptr->N * sizeof(double));

    return Cost;
}

double NonTrajOpt::LBFGSobjection(void* ptrObj,const double* optx,double* grad,const int n) {
    NonTrajOpt* OPTptr = reinterpret_cast<NonTrajOpt*>(ptrObj);
    double Cost = 0.0;

    OPTptr->iter_num++;
    std::cout << BLUE << "LBFGS iter_num: " << RESET <<OPTptr->iter_num << std::endl;

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
    // std::cout << " Cost: " << Cost << std::endl;
    std::memcpy(grad, _grad.data(), OPTptr->N * sizeof(double));
    return Cost;
}
// ##############################################################################################################
// Optimization Solvers #########################################################################################
// ##############################################################################################################
// OSQP Function 
bool NonTrajOpt::OSQPSolveDiv() {
    //// OSQP prepare
    bool flag = false;
    int succeed = 0;

    updateAeqbeqDiv();  // update MatAeqx, MatAeqy, MatAeqq, Vecbeqx, Vecbeqy, Vecbeqq
    updateMatQDiv();    // update MatkQ

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
    x_solver.settings()->setVerbosity(false);   // turn off output
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
        // step: 3 update the trajectory
        updateTraj();
        isDynamicFeasible(flag);
    }
    else {
        flag = false;
        std::cout << "[\033[31mOptimization\033[0m]: OSQP for [x,y,q] failed!!!" << std::endl;
    }

    return flag;
}


bool NonTrajOpt::OSQPSolve() {
    bool flag = false;

    // step: 0 update MatAeq, Vecbeq
    updateAeqbeq();    // update MatA, Vecb by time initT  
    updateMatQ();      // update MatQ by time initT

    Eigen::SparseMatrix<double> Hessian_sparse = MatQ.sparseView();

    Eigen::SparseMatrix<double> MatAeq_sparse = MatAeq.sparseView();

    Eigen::VectorXd gradient = Eigen::VectorXd::Zero(MatDim);
    Eigen::VectorXd lowerBound = Vecbeq;
    Eigen::VectorXd upperBound = Vecbeq;
    Eigen::VectorXd QPoptx = Eigen::VectorXd::Zero(MatDim);

    // instantiate the solver
    OsqpEigen::Solver solver;

    // step: 1 set the initial data of the QP solver
    solver.settings()->setWarmStart(true);

    // set the initial data of the QP solver
    // Ax < b
    solver.data()->setNumberOfVariables(MatDim);
    solver.data()->setNumberOfConstraints(EquDim);
    if(!solver.data()->setHessianMatrix(Hessian_sparse)) return flag;
    if(!solver.data()->setGradient(gradient)) return flag;
    if(!solver.data()->setLinearConstraintsMatrix(MatAeq_sparse)) return flag;
    if(!solver.data()->setLowerBound(lowerBound)) return flag;
    if(!solver.data()->setUpperBound(upperBound)) return flag;

    // instantiate the solver
    if(!solver.initSolver()) return flag;

    // step: 2 solve the QP problem
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
    double maxval = QPoptx.array().abs().maxCoeff();
    if (maxval > coeff_bound) {
        std::cout << YELLOW << "OSQP Solution is out of bound: " << maxval << RESET << std::endl;
        flag = false;
        return flag;
    }

    // step: 3 update the trajectory
    updateTraj();

    isDynamicFeasible(flag);

    return flag;
}

// NLopt Function
bool NonTrajOpt::NLoptSolve() {
    bool flag = false;
    nlopt::opt opt(nlopt::algorithm::LD_TNEWTON_PRECOND_RESTART, OptNum);
    // NLOPT_LD_LBFGS 
    // NLOPT_LD_TNEWTON_PRECOND_RESTART
    // NLOPT_LD_TNEWTON_PRECOND
    // NLOPT_LD_TNEWTON
    // NLOPT_LD_VAR1
    // // ❗❗❗ NLoptobjection 必须返回 static double 类型的值 且该类型只能声明时候有，定义函数的时候就不能写 static 了
    opt.set_min_objective(NonTrajOpt::NLoptobjection, this);

    std::vector<double> ub(OptNum);
    std::vector<double> lb(OptNum);
    
    Eigen::VectorXd c_upperb = Eigen::VectorXd::Constant(OptDof,coeff_bound);
    Eigen::VectorXd t_upperb = Eigen::VectorXd::Constant(N,TIME_TAU_UPPER_BOUND);
    Eigen::VectorXd c_lowerb = Eigen::VectorXd::Constant(OptDof,-coeff_bound);
    Eigen::VectorXd t_lowerb = Eigen::VectorXd::Constant(N,TIME_TAU_LOWER_BOUND);


    Eigen::VectorXd initc = Eigen::VectorXd::Constant(OptDof,INIT_COEFFS);
    Eigen::VectorXd initt = Eigen::VectorXd::Constant(N,INIT_TIME_TAU);

    opt.set_ftol_rel(nlopt_ftol_rel_);
    opt.set_xtol_rel(nlopt_xtol_rel_);
    opt.set_maxeval(nlopt_max_iteration_num_);
    opt.set_maxtime(nlopt_max_iteration_time_);

    std::copy(c_lowerb.data(), c_lowerb.data() + OptDof, lb.begin()); // 将 coeffs lower_bound 的值赋给 lb 的前 OptDof 个元素
    std::copy(c_upperb.data(), c_upperb.data() + OptDof, ub.begin()); // 将 coeffs upper_bound 的值赋给 ub 的前 OptDof 个元素
    if (TIME_OPTIMIZATION) {
        std::copy(t_lowerb.data(), t_lowerb.data() + N, lb.end() - N); // 将 time tau lower_bound 的值赋给 lb 的末尾 N 个元素
        std::copy(t_upperb.data(), t_upperb.data() + N, ub.end() - N); // 将 time tau upper_bound 的值赋给 ub 的末尾 N 个元素
    }

    if (BOUND_OPTIMIZATION) {
        // std::cout << "[\033[34mOptimization\033[0m]: nlopt bound optimization" << std::endl;
        // std::cout << "coeffs lower bound: " << c_lowerb.transpose() << std::endl;
        // std::cout << "coeffs upper bound: " << c_upperb.transpose() << std::endl;
        // std::cout << "time tau lower bound: " << t_lowerb.transpose() << std::endl;
        // std::cout << "time tau upper bound: " << t_upperb.transpose() << std::endl;
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

    std::cout << "init NLoptx: " ;
    for (const auto& element : NLoptx) {
        std::cout << element << " ";
    }
    std::cout << std::endl;

    auto nlopt_time_start = std::chrono::high_resolution_clock::now();
    try {

        nlopt::result result = opt.optimize(NLoptx, minf);
        std::cout << "[\033[34mOptimization\033[0m]: nlopt solver" << std::endl;
        std::cout << "NLopt iter: " << iter_num << std::endl;

        //// Time clock &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
        auto nlopt_time_end = std::chrono::high_resolution_clock::now();
        // std::chrono::microseconds获取微秒数 std::chrono::milliseconds获取毫秒数，或者使用std::chrono::seconds获取秒数
        auto nlopt_time_duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(nlopt_time_end - nlopt_time_start);
        // auto nlopt_time_duration_us = std::chrono::duration_cast<std::chrono::microseconds>(nlopt_time_end - nlopt_time_start);
        // auto nlopt_time_duration_s = std::chrono::duration_cast<std::chrono::seconds>(nlopt_time_end - nlopt_time_start);
        // std::cout << "NLopt time: " << nlopt_time_duration_us.count() << " us" << std::endl;
        std::cout << "NLopt time: " << nlopt_time_duration_ms.count() << " ms" << std::endl;
        // std::cout << "NLopt time: " << nlopt_time_duration_s.count() << " s" << std::endl;
        // std::cout << "NLopt minf: " << minf << std::endl;
        //// Debug result flag 
        std::cout << "NLopt result: " << result << " <=> ";
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

    if (flag) {
        Non_optx = Eigen::Map<const Eigen::VectorXd>(NLoptx.data(), NLoptx.size());
        // 更新优化变量
        updateOptVars(Non_optx);
        // 更新优化变量对应的矩阵 并求解 Ax=b
        updateOptAxb();
        // 将求解后的系数放入 Traj 中
        updateTraj();

        isDynamicFeasible(flag);
    }
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
    lbfgs_params.delta = 1e-6;  //1e-4
    lbfgs_params.max_iterations = 0;
    lbfgs_params.max_linesearch = 240; //default 60
    // lbfgs_params.min_step = 1e-16;
    lbfgs_params.xtol = 1e-5; //1e-4
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

    // std::cout << "Optx: ";
    // for (int i = 0; i < OptNum; i++) {
    //     std::cout << Optx[i] << " ";
    // }
    // std::cout << std::endl;

    // std::cout << "optx_: " ;
    // for (int i = 0; i < OptNum; i++) {
    //     std::cout << optx_[i] << " ";
    // }
    // std::cout << std::endl;

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

    isDynamicFeasible(flag);

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

inline void NonTrajOpt::isDynamicFeasible(bool &flag) {
    Eigen::Vector3d maxvel = Traj.getMaxVel();
    Eigen::Vector3d maxacc = Traj.getMaxAcc();
    Eigen::Vector2d linvel = maxvel.head(2);
    Eigen::Vector2d linacc = maxacc.head(2);
    double omevel = maxvel(2);
    double omeacc = maxacc(2);
    std::cout << "##########################################################################" << std::endl;
    std::cout << "maxlinvel: " << linvel.transpose() << "; maxomevel: " <<  omevel << std::endl;
    std::cout << "maxlinacc: " << linacc.transpose() << "; maxomeacc: " <<  omeacc << std::endl;
    std::cout << "linvelnorm: " << linvel.norm() << "; pv_max: " <<  pv_max/dyn_rate << std::endl;
    std::cout << "linaccnorm: " << linacc.norm() << "; pa_max: " <<  pa_max/dyn_rate << std::endl;
    std::cout << "omevel: " << omevel << "; wv_max: " <<  wv_max/dyn_rate << std::endl;
    std::cout << "omeacc: " << omeacc << "; wa_max: " <<  wa_max/dyn_rate << std::endl;
    std::cout << "##########################################################################" << std::endl;
    if (linvel.norm() > pv_max/dyn_rate || linacc.norm() > pa_max/dyn_rate || 
        abs(omevel) > wv_max/dyn_rate || abs(omeacc)  > wa_max/dyn_rate) {
        // flag = false;
    }
    else {
        // flag = true;
    }
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
    // gdsmotc.resize(3*10,N);
    // gdsmocostgrad.resize(6,N);
    for (int idx = 0; idx < N; idx++) {
        Eigen::VectorXd xcoef = Vecx.segment(idx*O*D, O);
        double xcost = getSegPolyCost(xcoef, idx);
        double xgradt = getSegPolyGradt(xcoef, idx);
        // std::cout << "xcoef: " << xcoef.transpose() << std::endl;
        ////&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
        // gdsmotc(0,idx) = 576.0 * xcoef(4) * xcoef(4);
        // gdsmotc(1,idx) = 5760.0 * xcoef(4) * xcoef(5)*T1(idx);
        // gdsmotc(2,idx) = 14400.0 * xcoef(5) * xcoef(5)*T2(idx);
        // gdsmotc(3,idx) = 17280.0 * xcoef(4) * xcoef(6)*T2(idx);
        // gdsmotc(4,idx) = 86400.0 * xcoef(5) * xcoef(6)*T3(idx);
        // gdsmotc(5,idx) = 40320.0 * xcoef(4) * xcoef(7)*T3(idx);
        // gdsmotc(6,idx) = 129600.0 * xcoef(6) * xcoef(6)*T4(idx);
        // gdsmotc(7,idx) = 201600.0 * xcoef(5) * xcoef(7)*T4(idx);
        // gdsmotc(8,idx) = 604800.0 * xcoef(6) * xcoef(7)*T5(idx);
        // gdsmotc(9,idx) = 705600.0 * xcoef(7) * xcoef(7)*T6(idx);
        
        getSegPolyGradc(xcoef, idx);
        Eigen::VectorXd ycoef = Vecx.segment(idx*O*D+O, O);
        double ycost = getSegPolyCost(ycoef, idx);
        double ygradt = getSegPolyGradt(ycoef, idx);
        // std::cout << "ycoef: " << ycoef.transpose() << std::endl;
        ////&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
        // gdsmotc(10,idx) = 576.0 * ycoef(4) * ycoef(4);
        // gdsmotc(11,idx) = 5760.0 * ycoef(4) * ycoef(5)*T1(idx);
        // gdsmotc(12,idx) = 14400.0 * ycoef(5) * ycoef(5)*T2(idx);
        // gdsmotc(13,idx) = 17280.0 * ycoef(4) * ycoef(6)*T2(idx);
        // gdsmotc(14,idx) = 86400.0 * ycoef(5) * ycoef(6)*T3(idx);
        // gdsmotc(15,idx) = 40320.0 * ycoef(4) * ycoef(7)*T3(idx);
        // gdsmotc(16,idx) = 129600.0 * ycoef(6) * ycoef(6)*T4(idx);
        // gdsmotc(17,idx) = 201600.0 * ycoef(5) * ycoef(7)*T4(idx);
        // gdsmotc(18,idx) = 604800.0 * ycoef(6) * ycoef(7)*T5(idx);
        // gdsmotc(19,idx) = 705600.0 * ycoef(7) * ycoef(7)*T6(idx);

        getSegPolyGradc(ycoef, idx);
        Eigen::VectorXd qcoef = Vecx.segment(idx*O*D+2*O, O);
        double qcost = getSegPolyCost(qcoef, idx);
        double qgradt = getSegPolyGradt(qcoef, idx);
        // std::cout << "qcoef: " << qcoef.transpose() << std::endl;
        ////&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
        // gdsmotc(20,idx) = 576.0 * qcoef(4) * qcoef(4);
        // gdsmotc(21,idx) = 5760.0 * qcoef(4) * qcoef(5)*T1(idx);
        // gdsmotc(22,idx) = 14400.0 * qcoef(5) * qcoef(5)*T2(idx);
        // gdsmotc(23,idx) = 17280.0 * qcoef(4) * qcoef(6)*T2(idx);
        // gdsmotc(24,idx) = 86400.0 * qcoef(5) * qcoef(6)*T3(idx);
        // gdsmotc(25,idx) = 40320.0 * qcoef(4) * qcoef(7)*T3(idx);
        // gdsmotc(26,idx) = 129600.0 * qcoef(6) * qcoef(6)*T4(idx);
        // gdsmotc(27,idx) = 201600.0 * qcoef(5) * qcoef(7)*T4(idx);
        // gdsmotc(28,idx) = 604800.0 * qcoef(6) * qcoef(7)*T5(idx);
        // gdsmotc(29,idx) = 705600.0 * qcoef(7) * qcoef(7)*T6(idx);

        getSegPolyGradc(qcoef, idx);
        gradc.segment(idx*O*D, O) = xcoef;
        gradc.segment(idx*O*D+O, O) = ycoef;
        gradc.segment(idx*O*D+2*O, O) = qcoef*wq;
        gradt(idx) = (xgradt + ygradt + qgradt*wq)*T1(idx);
        Cost += xcost + ycost + qcost*wq;
        // gdsmocostgrad(0,idx) = xgradt;
        // gdsmocostgrad(1,idx) = gdsmotc(0,idx) + gdsmotc(1,idx) + gdsmotc(2,idx) + gdsmotc(3,idx) + gdsmotc(4,idx) + 
        //                        gdsmotc(5,idx) + gdsmotc(6,idx) + gdsmotc(7,idx) + gdsmotc(8,idx) + gdsmotc(9,idx);
        // gdsmocostgrad(2,idx) = ygradt;
        // gdsmocostgrad(3,idx) = gdsmotc(10,idx) + gdsmotc(11,idx) + gdsmotc(12,idx) + gdsmotc(13,idx) + gdsmotc(14,idx) + 
        //                        gdsmotc(15,idx) + gdsmotc(16,idx) + gdsmotc(17,idx) + gdsmotc(18,idx) + gdsmotc(19,idx);
        // gdsmocostgrad(4,idx) = qgradt;
        // gdsmocostgrad(5,idx) = gdsmotc(20,idx) + gdsmotc(21,idx) + gdsmotc(22,idx) + gdsmotc(23,idx) + gdsmotc(24,idx) + 
        //                        gdsmotc(25,idx) + gdsmotc(26,idx) + gdsmotc(27,idx) + gdsmotc(28,idx) + gdsmotc(29,idx);
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
    // Eigen::VectorXi DiscreteNums = Traj.getDiscreteNums();
    // gdobstc.resize(10,(DiscreteNums.sum()+DiscreteNums.size()));
    // gdobstc.setZero();
    // gdobstimemat.resize(8,(DiscreteNums.sum()+DiscreteNums.size()));
    // gdobstimemat.setZero();
    // int gdidx = 0;
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
            // gdobstc(0,gdidx) = pos(0);
            // gdobstc(1,gdidx) = pos(1);
            // gdobstc(2,gdidx) = dist;
            // gdobstc(3,gdidx) = grad(0);
            // gdobstc(4,gdidx) = grad(1);
            // gdobstc(5,gdidx) = Tdt;
            // gdobstc(6,gdidx) = 1/std::pow(2*(dist),2) * vel.squaredNorm() * Tdt;
            // gdobstc(7,gdidx) = vel.squaredNorm();
            // gdobstc(8,gdidx) = 1/std::pow(2*(dist),2);
            // gdobstc(9,gdidx) = -2*std::pow(dist,3);
            // gdobstimemat.col(gdidx) = vectime;
            // gdidx++;
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
    // Eigen::VectorXi DiscreteNums = Traj.getDiscreteNums();
    // gddyntc.resize(14,(DiscreteNums.sum()+DiscreteNums.size()));
    // gddyntc.setZero();
    // int gdidx = 0;
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
            // gddyntc(0,gdidx) = Tdt;
            // gddyntc(1,gdidx) = deltaVel(0);
            // gddyntc(2,gdidx) = deltaVel(1);
            // gddyntc(3,gdidx) = deltaVel(2);
            // gddyntc(4,gdidx) = deltaAcc(0);
            // gddyntc(5,gdidx) = deltaAcc(1);
            // gddyntc(6,gdidx) = deltaAcc(2);
            // gddyntc(7,gdidx) = deltaVel.squaredNorm();
            // gddyntc(8,gdidx) = deltaAcc.squaredNorm();
            // gddyntc(9,gdidx) = 2*deltaVel.dot(acc);
            // gddyntc(10,gdidx) = 2*deltaAcc.dot(jer);
            // gddyntc(11,gdidx) = vel.squaredNorm();
            // gddyntc(12,gdidx) = acc.squaredNorm();
            // gddyntc(13,gdidx) = jer.squaredNorm();
            // gdidx++;
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
    // Eigen::VectorXi DiscreteNums = Traj.getDiscreteNums();
    // gdovatc.resize(17,(DiscreteNums.sum()+DiscreteNums.size()));
    // gdovatc.setZero();
    // int gdidx = 0;
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
            // gdovatc(0,gdidx) = Tdt;
            // gdovatc(1,gdidx) = vel_B(0);
            // gdovatc(2,gdidx) = vel_B(1);
            // gdovatc(3,gdidx) = vel_B2;
            // gdovatc(4,gdidx) = Rmatrix(0,0);
            // gdovatc(5,gdidx) = Rmatrix(0,1);
            // gdovatc(6,gdidx) = Rmatrix(1,0);
            // gdovatc(7,gdidx) = Rmatrix(1,1);
            // gdovatc(8,gdidx) = (2 * vel_B(1) * (acc_B(1) - cosyaw * vel(0) * vel(2) - sinyaw * vel(1) * vel(2)) / VERDIT_VEL2 +
            //                    2 * vel_B(0) * (acc_B(0) + cosyaw * vel(1) * vel(2) - sinyaw * vel(0) * vel(2)) / ORIEN_VEL2) * Tdt;
            // gdovatc(9,gdidx) = (2 * cosyaw * vel_B(0)/ORIEN_VEL2 - 2 * sinyaw * vel_B(1) / VERDIT_VEL2);
            // gdovatc(10,gdidx) = (2 * sinyaw * vel_B(0)/ORIEN_VEL2 + 2 * cosyaw * vel_B(1) / VERDIT_VEL2);
            // gdovatc(11,gdidx) = (2 * vel_B(0) * vel_B(1) / ORIEN_VEL2 - 2 * vel_B(1) * vel_B(0) / VERDIT_VEL2);
            // gdovatc(12,gdidx) = acc_B(0);
            // gdovatc(13,gdidx) = acc_B(1);
            // gdovatc(14,gdidx) = acc_B.squaredNorm();
            // gdovatc(15,gdidx) = acc_B(0)*acc_B(0) + acc_B(1)*acc_B(1);
            // gdovatc(16,gdidx) = yaw_q;
            // gdidx++;
        }
        gradc.segment(idx*O*D, O)       = xgrad;
        gradc.segment(idx*O*D+O, O)     = ygrad;
        gradc.segment(idx*O*D+2*O, O)   = qgrad;
    }
}

void NonTrajOpt::calcuOvaConstriants() {

}
