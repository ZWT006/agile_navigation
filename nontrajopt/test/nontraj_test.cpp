/*
 * @Author: wentao zhang && zwt190315@163.com
 * @Date: 2023-07-07
 * @LastEditTime: 2023-07-13
 * @Description: trajectory optimization class test
 * @reference: 
 * 
 * Copyright (c) 2023 by wentao zhang, All Rights Reserved. 
 */

#include <gtest/gtest.h>
#include <iostream>
#include <Eigen/Eigen>
#include <nontrajopt/nontrajopt.hpp>
#include <nontrajopt/Eigencsv.hpp>

// using namespace NonTrajOptSpace;
EigenCSV eigen_csv;

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    // ros::init (argc,argv, "tester" );
    // ros::NodeHandle nh;
    // testing::GTEST_FLAG(filter) = "NonTrajOptTest.OSQPSolve";
    // testing::GTEST_FLAG(filter) = "NonTrajOptTest.NLoptSolver";
    testing::GTEST_FLAG(filter) = "NonTrajOptTest.NontrajOptClass";
    return RUN_ALL_TESTS();
}

TEST(NonTrajOptTest, NontrajOptClass) {
    NonTrajOpt nontrajopt;
    OptParas paras;
    paras.lambda_smo = 0.1;
    paras.lambda_obs = 0.1;
    paras.lambda_dyn = 0.1;
    paras.lambda_tim = 0.1;
    paras.lambda_ova = 0.1;

    paras.wq = 0.8;
    paras.dist_th = 0.1;
    paras.discredist = 0.1;

    paras.pv_max = 2.0;
    paras.pa_max = 2.0;
    paras.wv_max = 2.0;
    paras.wa_max = 2.0;

    paras.ORIEN_VEL = 1.0;
    paras.VERDIT_VEL = 2.0;
    paras.OVAL_TH = 0.8;

    paras.SMO_SWITCH = true;
    paras.OBS_SWITCH = false;
    paras.DYN_SWITCH = false;
    paras.TIM_SWITCH = false;
    paras.OVA_SWITCH = false;

    paras.nlopt_max_iteration_num_ = 100;
    paras.nlopt_max_iteration_time_ = 0.1;

    paras.TIME_OPTIMIZATION = true;
    paras.REDUCE_ORDER = true;

    nontrajopt.initParameter(paras);
    nontrajopt.reset(3);

    Eigen::Matrix3Xd _waypoints;
    _waypoints.resize(3, 4);
    // 输入数据会按列的顺序进行排列
    _waypoints <<   2.0,    2.2902, 2.5465, 3.0,   
                    4.5,    4.9912, 5.3379, 5.5,
                    1.5708, 0.7264, 0.2336, 0.0;

    // std::cout << "_waypoints: " << _waypoints << std::endl;

    Eigen::VectorXd _initT;
    _initT.resize(3);
    _initT << 0.7757, 0.2853, 0.6976;

    Eigen::Matrix<double, 3, 4> _startStates;
    _startStates.resize(3, 4);
    _startStates.setZero();
    _startStates.col(0) = _waypoints.col(0);

    Eigen::Matrix<double, 3, 4> _endStates;
    _endStates.resize(3, 4);
    _endStates.setZero();
    _endStates.col(0) = _waypoints.col(3);

    nontrajopt.initWaypoints(_waypoints, _initT, _startStates, _endStates);
    Eigen::VectorXd _optx;
    _optx.resize(nontrajopt.OptDof + 3);
    _optx.setZero();
    
    // std::cout << "_initT.array().log(): " << _initT.array().log() << std::endl;
    
    _optx.tail(3) = _initT.array().log();
    
    // std::cout << "_optx.array().exp():" << _optx.tail(3).array().exp() << std::endl;

    Eigen::VectorXd _coeff;
    _coeff.resize(nontrajopt.MatDim);
    _coeff<<2.0, 0.0, 0.0, 0.0, 2.5996, -4.7631, 3.0521, -0.6598,
            4.5, 0.0, 0.0, 0.0, 0.1842, 0.5642, -0.625, 0.1577,
            1.5708, 0.0, 0.0, 0.0, -0.4839, -0.9201, 1.2507, -0.356,
            2.2902, 0.3699, 0.6392, 1.4873, 0.2336, -4.8276, 2.3715, 0.0449,
            4.9912, 1.088, 0.1168, -1.1587, -0.3682, 0.8588, 0.7065, -0.7962,
            0.7264, -1.65, 0.252, 1.7995, 0.4458, -10.3844, 17.0949, -7.4132,
            2.5465, 1.1204, 0.8292, -2.0726, -3.4322, 2.55, 7.6172, -6.397;
            5.3379, 0.7535, -0.9336, -0.4246, 1.241, 0.2793, -1.2553, 0.4926,
            0.2336, -1.0674, 1.05, 0.4888, 4.0399, -16.4143, 18.8061, -7.1919;

    // coeffs x
    // 2.0, 0.0, 0.0, 0.0, 2.5996, -4.7631, 3.0521, -0.6598,
    // 2.2902, 0.3699, 0.6392, 1.4873, 0.2336, -4.8276, 2.3715, 0.0449,
    // 2.5465, 1.1204, 0.8292, -2.0726, -3.4322, 2.55, 7.6172, -6.397;

    // coeffs y
    // 4.5, 0.0, 0.0, 0.0, 0.1842, 0.5642, -0.625, 0.1577,
    // 4.9912, 1.088, 0.1168, -1.1587, -0.3682, 0.8588, 0.7065, -0.7962,
    // 5.3379, 0.7535, -0.9336, -0.4246, 1.241, 0.2793, -1.2553, 0.4926,

    // coeffs z
    // 1.5708, 0.0, 0.0, 0.0, -0.4839, -0.9201, 1.2507, -0.356,
    // 0.7264, -1.65, 0.252, 1.7995, 0.4458, -10.3844, 17.0949, -7.4132,
    // 0.2336, -1.0674, 1.05, 0.4888, 4.0399, -16.4143, 18.8061, -7.1919,

    nontrajopt.getReduceOrder(_coeff);

    _optx.head(nontrajopt.OptDof) = _coeff;

    nontrajopt.updateOptVars(_optx);

    nontrajopt.updateOptAxb();    // update MatA, Vecb by time initT

    std::cout << "OptDot : " << nontrajopt.OptDof << std::endl;
    std::cout << "EquDim : " << nontrajopt.EquDim << std::endl;

    // eigen_csv.WriteMatrix(nontrajopt.MatQ, "/home/zwt/Documents/MatQ.csv");
    // std::cout << "MatQ.eigenvalues()" << std::endl;
    // std::cout << nontrajopt.MatQ.eigenvalues().real() << std::endl;
    // std::cout << "nontrajopt.MatQ: " << std::endl;
    // eigen_csv.WriteMatrix(nontrajopt.MatAeq, "/home/zwt/Documents/MatAeq.csv");
    // std::cout << "nontrajopt.MatAeq: " << std::endl;
    // eigen_csv.WriteVector(nontrajopt.Vecbeq, "/home/zwt/Documents/Vecbeq.csv");
    // std::cout << "nontrajopt.Vecbeq: " << std::endl;

}

TEST(NonTrajOptTest, NLoptSolver) {
    NonTrajOpt nontrajopt;
    OptParas paras;
    paras.lambda_smo = 0.1;
    paras.lambda_obs = 0.1;
    paras.lambda_dyn = 0.1;
    paras.lambda_tim = 0.1;
    paras.lambda_ova = 0.1;

    paras.wq = 0.8;
    paras.dist_th = 0.1;
    paras.discredist = 0.1;

    paras.pv_max = 2.0;
    paras.pa_max = 2.0;
    paras.wv_max = 2.0;
    paras.wa_max = 2.0;

    paras.ORIEN_VEL = 1.0;
    paras.VERDIT_VEL = 2.0;
    paras.OVAL_TH = 0.8;

    paras.SMO_SWITCH = true;
    paras.OBS_SWITCH = false;
    paras.DYN_SWITCH = false;
    paras.TIM_SWITCH = false;
    paras.OVA_SWITCH = false;

    paras.nlopt_max_iteration_num_ = 100;
    paras.nlopt_max_iteration_time_ = 0.1;

    nontrajopt.initParameter(paras);
    nontrajopt.reset(3);

    Eigen::Matrix3Xd _waypoints;
    _waypoints.resize(3, 4);
    // 输入数据会按列的顺序进行排列
    _waypoints <<   2.0,    2.2902, 2.5465, 3.0,   
                    4.5,    4.9912, 5.3379, 5.5,
                    1.5708, 0.7264, 0.2336, 0.0;

    // std::cout << "_waypoints: " << _waypoints << std::endl;

    Eigen::VectorXd _initT;
    _initT.resize(3);
    _initT << 0.7757, 0.2853, 0.6976;

    Eigen::Matrix<double, 3, 4> _startStates;
    _startStates.resize(3, 4);
    _startStates.setZero();
    _startStates.col(0) = _waypoints.col(0);

    Eigen::Matrix<double, 3, 4> _endStates;
    _endStates.resize(3, 4);
    _endStates.setZero();
    _endStates.col(0) = _waypoints.col(3);

    nontrajopt.initWaypoints(_waypoints, _initT, _startStates, _endStates);

    nontrajopt.updateAeqbeq();    // update MatA, Vecb by time initT  
    nontrajopt.updateMatQ();      // update MatQ by time initT

    eigen_csv.WriteMatrix(nontrajopt.MatQ, "/home/zwt/Documents/MatQ.csv");
    std::cout << "MatQ.eigenvalues()" << std::endl;
    std::cout << nontrajopt.MatQ.eigenvalues().real() << std::endl;
    std::cout << "nontrajopt.MatQ: " << std::endl;
    eigen_csv.WriteMatrix(nontrajopt.MatAeq, "/home/zwt/Documents/MatAeq.csv");
    std::cout << "nontrajopt.MatAeq: " << std::endl;
    eigen_csv.WriteVector(nontrajopt.Vecbeq, "/home/zwt/Documents/Vecbeq.csv");
    std::cout << "nontrajopt.Vecbeq: " << std::endl;

    // nontrajopt.NLoptSolve();
}

/*
 * @description: test the function of OSQPSolve
 * @param {*}
 * @return {*}
 */
TEST(NonTrajOptTest, OSQPSolve) {
    NonTrajOpt nontrajopt;
    OptParas paras;
    paras.lambda_smo = 0.1;
    paras.lambda_obs = 0.1;
    paras.lambda_dyn = 0.1;
    paras.lambda_tim = 0.1;
    paras.lambda_ova = 0.1;

    paras.wq = 0.8;
    paras.dist_th = 0.1;
    paras.discredist = 0.1;

    paras.pv_max = 0.1;
    paras.pa_max = 0.1;
    paras.wv_max = 0.1;
    paras.wa_max = 0.1;

    paras.ORIEN_VEL = 1.0;
    paras.VERDIT_VEL = 2.0;
    paras.OVAL_TH = 0.8;

    paras.SMO_SWITCH = true;
    paras.OBS_SWITCH = true;
    paras.DYN_SWITCH = true;
    paras.TIM_SWITCH = true;
    paras.OVA_SWITCH = true;

    paras.nlopt_max_iteration_num_ = 100;
    paras.nlopt_max_iteration_time_ = 0.1;

    nontrajopt.initParameter(paras);
    nontrajopt.reset(3);

    Eigen::Matrix3Xd _waypoints;
    _waypoints.resize(3, 4);
    // 输入数据会按列的顺序进行排列
    _waypoints <<   2.0,    2.2902, 2.5465, 3.0,   
                    4.5,    4.9912, 5.3379, 5.5,
                    1.5708, 0.7264, 0.2336, 0.0;

    // std::cout << "_waypoints: " << _waypoints << std::endl;

    Eigen::VectorXd _initT;
    _initT.resize(3);
    _initT << 0.7757, 0.2853, 0.6976;

    Eigen::Matrix<double, 3, 4> _startStates;
    _startStates.resize(3, 4);
    _startStates.setZero();
    _startStates.col(0) = _waypoints.col(0);

    Eigen::Matrix<double, 3, 4> _endStates;
    _endStates.resize(3, 4);
    _endStates.setZero();
    _endStates.col(0) = _waypoints.col(3);

    nontrajopt.initWaypoints(_waypoints, _initT, _startStates, _endStates);

    nontrajopt.updateAeqbeq();    // update MatA, Vecb by time initT  
    nontrajopt.updateMatQ();      // update MatQ by time initT

    eigen_csv.WriteMatrix(nontrajopt.MatQ, "/home/zwt/Documents/MatQ.csv");
    std::cout << "nontrajopt.MatQ: " << std::endl;
    eigen_csv.WriteMatrix(nontrajopt.MatAeq, "/home/zwt/Documents/MatAeq.csv");
    std::cout << "nontrajopt.MatAeq: " << std::endl;
    eigen_csv.WriteVector(nontrajopt.Vecbeq, "/home/zwt/Documents/Vecbeq.csv");
    std::cout << "nontrajopt.Vecbeq: " << std::endl;

    nontrajopt.OSQPSolve();
}