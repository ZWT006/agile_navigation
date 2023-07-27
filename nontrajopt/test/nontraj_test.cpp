/*
 * @Author: wentao zhang && zwt190315@163.com
 * @Date: 2023-07-07
 * @LastEditTime: 2023-07-26
 * @Description: trajectory optimization class test
 * @reference: 
 * 
 * Copyright (c) 2023 by wentao zhang, All Rights Reserved. 
 */

#include <gtest/gtest.h>
#include <iostream>
#include <Eigen/Eigen>

#include <nontrajopt/Eigencsv.hpp>

#include "matplotlibcpp.h"
#include <opencv2/opencv.hpp>


#include <nontrajopt/nontrajopt.hpp>
// #include <nontrajopt/nontrajoptclass.h>
namespace plt = matplotlibcpp;

// using namespace NonTrajOptSpace;
EigenCSV eigen_csv;

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    // ros::init (argc,argv, "tester" );
    // ros::NodeHandle nh;
    // testing::GTEST_FLAG(filter) = "NonTrajOptTest.NontrajOptClass";
    testing::GTEST_FLAG(filter) = "NonTrajOptTest.NLoptSolver";
    // testing::GTEST_FLAG(filter) = "NonTrajOptTest.LBFGSSolver";
    // testing::GTEST_FLAG(filter) = "NonTrajOptTest.OSQPSolve";
    // testing::GTEST_FLAG(filter) = "NonTrajOptTest.CostGradDebug";
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

    paras.TIME_OPTIMIZATION = false;
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
            2.5465, 1.1204, 0.8292, -2.0726, -3.4322, 2.55, 7.6172, -6.397,
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

    nontrajopt.updateTraj();

    // 设置 EDF 的参数
    nontrajopt.setEDFMap(12,8,0.01,0.01);

    // 读取地图
    nontrajopt.readEDFMap("/home/zwt/motion_ws/src/navigation/grid_path_searcher/map/map10.png",30);

    // cv::imshow("map", nontrajopt.edfmap._edfmap_A);
    // cv::waitKey(0);

    // double dist = 0.0;
    // int idx = 0, idy = 0;
    // std::cout << "idx:     ,idy:     ,dist:     " << std::endl;
    // idy = 240; idx = 200;
    // dist = nontrajopt.edfmap.getDist(idx,idy);
    // std::cout << idx << "  " << idy << "   " << dist << std::endl;
    // idy = 255; idx = 200;
    // dist = nontrajopt.edfmap.getDist(idx,idy);
    // std::cout << idx << "  " << idy << "   " << dist << std::endl;
    // idy = 265; idx = 200;
    // dist = nontrajopt.edfmap.getDist(idx,idy);
    // std::cout << idx << "  " << idy << "   " << dist << std::endl;
    // idy = 275; idx = 200;
    // dist = nontrajopt.edfmap.getDist(idx,idy);
    // std::cout << idx << "  " << idy << "   " << dist << std::endl;
    // idy = 300; idx = 200;
    // dist = nontrajopt.edfmap.getDist(idx,idy);
    // std::cout << idx << "  " << idy << "   " << dist << std::endl;
    // idy = 350; idx = 200;
    // dist = nontrajopt.edfmap.getDist(idx,idy);
    // std::cout << idx << "  " << idy << "   " << dist << std::endl;

    double dt = 0.01;
    double allT = nontrajopt.Traj.getTotalDuration();
    int num = allT / dt;
    std::vector<double> time(num);
    std::vector<double> px(num), py(num), pq(num);
    std::vector<double> vx(num), vy(num), vq(num);
    Eigen::MatrixXd _traj;
    _traj.resize(num, 7);
    for (int i = 0; i < num; i++) {
        double t = i * dt;
        time[i] = t;
        px[i] = nontrajopt.Traj.getPos(t)(0);
        py[i] = nontrajopt.Traj.getPos(t)(1);
        pq[i] = nontrajopt.Traj.getPos(t)(2);
        vx[i] = nontrajopt.Traj.getVel(t)(0);
        vy[i] = nontrajopt.Traj.getVel(t)(1);
        vq[i] = nontrajopt.Traj.getVel(t)(2);
        _traj.row(i) << t, px[i], py[i], pq[i], vx[i], vy[i], vq[i];
    }

    // eigen_csv.WriteMatrix(_traj, "/home/zwt/Documents/TRAJ_STATE.csv");

    ////TODO: 感觉这里可能还有问题 plot 出来的图像 y轴数值非常大 感觉不科学呀
    plt::figure_size(1200, 800);
    // Plot line from given x and y data. Color is selected automatically.
    plt::plot(time, px,"r-");
    plt::plot(time, py,"g-");
    plt::plot(time, pq,"b-");
    // Add graph title
    plt::title("position figure");
    // Enable legend.
    plt::legend();
    // plt::show();

    // plt::figure(2);
    plt::figure_size(1200, 800);
    // Plot line from given x and y data. Color is selected automatically.
    plt::plot(time, vx,"r-");
    plt::plot(time, vy,"g-");
    plt::plot(time, vq,"b-");
    // Add graph title
    plt::title("velocity figure");
    // Enable legend.
    // plt::legend({"vx", "vy", "vq"});
    plt::show(); // 显示图像(阻塞 并且显示所有图像)

}

TEST(NonTrajOptTest, NLoptSolver) {
    NonTrajOpt nontrajopt;
    OptParas paras;
    paras.lambda_smo = 0.1;
    paras.lambda_obs = 1.0;
    paras.lambda_dyn = 10.0;
    paras.lambda_tim = 8000.0;
    paras.lambda_ova = 10.0;

    paras.wq = 0.8;
    paras.dist_th = 0.5;
    paras.discredist = 0.05;

    //// MATLAB 1.5*ref_vel
    paras.pv_max = 2.1;
    paras.pa_max = 2.1;
    paras.wv_max = 2.1;
    paras.wa_max = 2.1;
    paras.dyn_rate = 0.8;

    paras.ORIEN_VEL = 2.0;
    paras.VERDIT_VEL = 1.0;
    paras.OVAL_TH = 0.8;

    paras.SMO_SWITCH = true;
    paras.OBS_SWITCH = true;
    paras.DYN_SWITCH = true;
    paras.TIM_SWITCH = true;
    paras.OVA_SWITCH = true;

    paras.TIME_OPTIMIZATION = true;
    paras.REDUCE_ORDER = true;
    paras.BOUND_OPTIMIZATION = true;

    paras.INIT_OPT_VALUE  = true;

    paras.nlopt_max_iteration_num_ = 1000;
    paras.nlopt_max_iteration_time_ = 10.0;

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
    _initT << 1.2063, 0.3520, 0.6880;

    Eigen::Matrix<double, 3, 4> _startStates;
    _startStates.resize(3, 4);
    _startStates.setZero();
    _startStates.col(0) = _waypoints.col(0);

    Eigen::Matrix<double, 3, 4> _endStates;
    _endStates.resize(3, 4);
    _endStates.setZero();
    _endStates.col(0) = _waypoints.col(3);

    nontrajopt.initWaypoints(_waypoints, _initT, _startStates, _endStates);

    std::cout << "OptDof: " << nontrajopt.OptDof << std::endl;
    std::cout << "MatDim: " << nontrajopt.MatDim << std::endl;
    std::cout << "OptNum: " << nontrajopt.OptNum << std::endl;

    Eigen::VectorXd _optx;
    _optx.resize(nontrajopt.OptDof + 3);
    _optx.setZero();    
    _optx.tail(3) = _initT.array().log();
    Eigen::VectorXd _coeff;
    _coeff.resize(nontrajopt.MatDim);
    _coeff  << 2.0, 0.0,    0.0,    0.0,    2.5996, -4.7631, 3.0521, -0.6598,
            4.5,    0.0,    0.0,    0.0,    0.1842, 0.5642, -0.625, 0.1577,
            1.5708, 0.0,    0.0,    0.0,    -0.4839,-0.9201,1.2507, -0.356,
            2.2902, 0.3699, 0.6392, 1.4873, 0.2336, -4.8276,2.3715, 0.0449,
            4.9912, 1.088,  0.1168, -1.1587,-0.3682,0.8588, 0.7065, -0.7962,
            0.7264, -1.65,  0.252,  1.7995, 0.4458, -10.3844,17.0949, -7.4132,
            2.5465, 1.1204, 0.8292, -2.0726,-3.4322,2.55,   7.6172, -6.397,
            5.3379, 0.7535, -0.9336,-0.4246,1.241,  0.2793, -1.2553, 0.4926,
            0.2336, -1.0674,1.05,   0.4888, 4.0399, -16.4143, 18.8061, -7.1919;

    EXPECT_EQ(_coeff.size(), nontrajopt.MatDim);

    // std::cout << std::boolalpha ;
    // for (int i = 0; i < int(nontrajopt.optFlag.size()); i++) {
    //     std::cout << nontrajopt.optFlag[i] << " ";
    // }
    // std::cout << std::endl;

    nontrajopt.getReduceOrder(_coeff);
    
    _optx.head(nontrajopt.OptDof) = _coeff;

    nontrajopt.updateOptVars(_optx);

    nontrajopt.updateOptAxb();    // update MatA, Vecb by time initT

    nontrajopt.updateTraj();

    // 设置 EDF 的参数 map_width, map_height, map_resolution, mini_dist
    nontrajopt.setEDFMap(12,8,0.01,0.01);

    // 读取地图
    nontrajopt.readEDFMap("/home/zwt/motion_ws/src/navigation/grid_path_searcher/map/map10.png",0);

    std::cout << std::boolalpha << "SMO_SWITCH: " << nontrajopt.SMO_SWITCH << std::endl;
    std::cout << std::boolalpha << "OBS_SWITCH: " << nontrajopt.OBS_SWITCH << std::endl;
    std::cout << std::boolalpha << "DYN_SWITCH: " << nontrajopt.DYN_SWITCH << std::endl;
    std::cout << std::boolalpha << "TIM_SWITCH: " << nontrajopt.TIM_SWITCH << std::endl;
    std::cout << std::boolalpha << "OVA_SWITCH: " << nontrajopt.OVA_SWITCH << std::endl;

    std::cout << std::boolalpha << "TIME_OPTIMIZATION: " << nontrajopt.TIME_OPTIMIZATION << std::endl;
    std::cout << std::boolalpha << "REDUCE_ORDER: " << nontrajopt.REDUCE_ORDER << std::endl;
    std::cout << std::boolalpha << "INIT_OPT_VALUE: " << nontrajopt.INIT_OPT_VALUE << std::endl;
    std::cout << std::boolalpha << "BOUND_OPTIMIZATION: " << nontrajopt.BOUND_OPTIMIZATION << std::endl;

    std::cout << "MaxIterNum: " << nontrajopt.nlopt_max_iteration_num_ << std::endl;
    std::cout << "MaxIterTime: " << std::setprecision(2) << nontrajopt.nlopt_max_iteration_time_ << std::endl;

    bool succeed = nontrajopt.NLoptSolve();

    if (succeed)
        std::cout << GREEN << "NLoptSolve succeed! Happy!!!" << RESET << std::endl;
    else
        std::cout << RED << "NLoptSolve failed! Sad!!!" << RESET << std::endl;

    double dt = 0.01;
    double allT = nontrajopt.Traj.getTotalDuration();
    int num = allT / dt;
    std::vector<double> time(num);
    std::vector<double> px(num), py(num), pq(num);
    std::vector<double> vx(num), vy(num), vq(num);
    Eigen::MatrixXd _traj;
    _traj.resize(num, 7);
    for (int i = 0; i < num; i++) {
        double t = i * dt;
        time[i] = t;
        px[i] = nontrajopt.Traj.getPos(t)(0);
        py[i] = nontrajopt.Traj.getPos(t)(1);
        pq[i] = nontrajopt.Traj.getPos(t)(2);
        vx[i] = nontrajopt.Traj.getVel(t)(0);
        vy[i] = nontrajopt.Traj.getVel(t)(1);
        vq[i] = nontrajopt.Traj.getVel(t)(2);
        _traj.row(i) << t, px[i], py[i], pq[i], vx[i], vy[i], vq[i];
    }

    // eigen_csv.WriteMatrix(_traj, "/home/zwt/Documents/TRAJ_STATE.csv");

    ////TODO: 感觉这里可能还有问题 plot 出来的图像 y轴数值非常大 感觉不科学呀
    plt::figure_size(1200, 800);
    // Plot line from given x and y data. Color is selected automatically.
    plt::plot(time, px,"r-");
    plt::plot(time, py,"g-");
    plt::plot(time, pq,"b-");
    // Add graph title
    plt::title("position figure");
    // Enable legend.
    // plt::legend();
    // plt::show();

    // plt::figure(2);
    plt::figure_size(1200, 800);
    // Plot line from given x and y data. Color is selected automatically.
    plt::plot(time, vx,"r-");
    plt::plot(time, vy,"g-");
    plt::plot(time, vq,"b-");
    // Add graph title
    plt::title("velocity figure");
    // Enable legend.
    // plt::legend({"vx", "vy", "vq"});
    plt::show(); // 显示图像(阻塞 并且显示所有图像)

}

TEST(NonTrajOptTest, LBFGSSolver) {
    NonTrajOpt nontrajopt;
    OptParas paras;
    paras.lambda_smo = 0.1;
    paras.lambda_obs = 0.1;
    paras.lambda_dyn = 0.1;
    paras.lambda_tim = 2000.0;
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

    paras.TIME_OPTIMIZATION = false;
    paras.REDUCE_ORDER = true;
    paras.BOUND_OPTIMIZATION = true;

    paras.INIT_OPT_VALUE  = true;

    paras.nlopt_max_iteration_num_ = 1000;
    paras.nlopt_max_iteration_time_ = 10.0;

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

    std::cout << "OptDof: " << nontrajopt.OptDof << std::endl;
    std::cout << "MatDim: " << nontrajopt.MatDim << std::endl;
    std::cout << "OptNum: " << nontrajopt.OptNum << std::endl;

    Eigen::VectorXd _optx;
    _optx.resize(nontrajopt.OptDof + 3);
    _optx.setZero();    
    _optx.tail(3) = _initT.array().log();
    Eigen::VectorXd _coeff;
    _coeff.resize(nontrajopt.MatDim);
    _coeff  << 2.0, 0.0,    0.0,    0.0,    2.5996, -4.7631, 3.0521, -0.6598,
            4.5,    0.0,    0.0,    0.0,    0.1842, 0.5642, -0.625, 0.1577,
            1.5708, 0.0,    0.0,    0.0,    -0.4839,-0.9201,1.2507, -0.356,
            2.2902, 0.3699, 0.6392, 1.4873, 0.2336, -4.8276,2.3715, 0.0449,
            4.9912, 1.088,  0.1168, -1.1587,-0.3682,0.8588, 0.7065, -0.7962,
            0.7264, -1.65,  0.252,  1.7995, 0.4458, -10.3844,17.0949, -7.4132,
            2.5465, 1.1204, 0.8292, -2.0726,-3.4322,2.55,   7.6172, -6.397,
            5.3379, 0.7535, -0.9336,-0.4246,1.241,  0.2793, -1.2553, 0.4926,
            0.2336, -1.0674,1.05,   0.4888, 4.0399, -16.4143, 18.8061, -7.1919;

    EXPECT_EQ(_coeff.size(), nontrajopt.MatDim);

    // std::cout << std::boolalpha ;
    // for (int i = 0; i < int(nontrajopt.optFlag.size()); i++) {
    //     std::cout << nontrajopt.optFlag[i] << " ";
    // }
    // std::cout << std::endl;

    nontrajopt.getReduceOrder(_coeff);
    
    _optx.head(nontrajopt.OptDof) = _coeff;

    nontrajopt.updateOptVars(_optx);

    nontrajopt.updateOptAxb();    // update MatA, Vecb by time initT

    nontrajopt.updateTraj();

    // 设置 EDF 的参数 map_width, map_height, map_resolution, mini_dist
    nontrajopt.setEDFMap(12,8,0.01,0.01);

    // 读取地图
    nontrajopt.readEDFMap("/home/zwt/motion_ws/src/navigation/grid_path_searcher/map/map10.png",0);

    std::cout << std::boolalpha << "SMO_SWITCH: " << nontrajopt.SMO_SWITCH << std::endl;
    std::cout << std::boolalpha << "OBS_SWITCH: " << nontrajopt.OBS_SWITCH << std::endl;
    std::cout << std::boolalpha << "DYN_SWITCH: " << nontrajopt.DYN_SWITCH << std::endl;
    std::cout << std::boolalpha << "TIM_SWITCH: " << nontrajopt.TIM_SWITCH << std::endl;
    std::cout << std::boolalpha << "OVA_SWITCH: " << nontrajopt.OVA_SWITCH << std::endl;

    std::cout << std::boolalpha << "TIME_OPTIMIZATION: " << nontrajopt.TIME_OPTIMIZATION << std::endl;
    std::cout << std::boolalpha << "REDUCE_ORDER: " << nontrajopt.REDUCE_ORDER << std::endl;
    std::cout << std::boolalpha << "INIT_OPT_VALUE: " << nontrajopt.INIT_OPT_VALUE << std::endl;
    std::cout << std::boolalpha << "BOUND_OPTIMIZATION: " << nontrajopt.BOUND_OPTIMIZATION << std::endl;

    std::cout << "MaxIterNum: " << nontrajopt.nlopt_max_iteration_num_ << std::endl;
    std::cout << "MaxIterTime: " << std::setprecision(2) << nontrajopt.nlopt_max_iteration_time_ << std::endl;

    bool succeed = nontrajopt.LBFGSSolve();

    if (succeed)
        std::cout << GREEN << "LBFGSSolve succeed! Happy!!!" << RESET << std::endl;
    else
        std::cout << RED << "LBFGSSolve failed! Sad!!!" << RESET << std::endl;

        double dt = 0.01;
    double allT = nontrajopt.Traj.getTotalDuration();
    int num = allT / dt;
    std::vector<double> time(num);
    std::vector<double> px(num), py(num), pq(num);
    std::vector<double> vx(num), vy(num), vq(num);
    Eigen::MatrixXd _traj;
    _traj.resize(num, 7);
    for (int i = 0; i < num; i++) {
        double t = i * dt;
        time[i] = t;
        px[i] = nontrajopt.Traj.getPos(t)(0);
        py[i] = nontrajopt.Traj.getPos(t)(1);
        pq[i] = nontrajopt.Traj.getPos(t)(2);
        vx[i] = nontrajopt.Traj.getVel(t)(0);
        vy[i] = nontrajopt.Traj.getVel(t)(1);
        vq[i] = nontrajopt.Traj.getVel(t)(2);
        _traj.row(i) << t, px[i], py[i], pq[i], vx[i], vy[i], vq[i];
    }

    // eigen_csv.WriteMatrix(_traj, "/home/zwt/Documents/TRAJ_STATE.csv");

    ////TODO: 感觉这里可能还有问题 plot 出来的图像 y轴数值非常大 感觉不科学呀
    plt::figure_size(1200, 800);
    // Plot line from given x and y data. Color is selected automatically.
    plt::plot(time, px,"r-");
    plt::plot(time, py,"g-");
    plt::plot(time, pq,"b-");
    // Add graph title
    plt::title("position figure");
    // Enable legend.
    // plt::legend();
    // plt::show();

    // plt::figure(2);
    plt::figure_size(1200, 800);
    // Plot line from given x and y data. Color is selected automatically.
    plt::plot(time, vx,"r-");
    plt::plot(time, vy,"g-");
    plt::plot(time, vq,"b-");
    // Add graph title
    plt::title("velocity figure");
    // Enable legend.
    // plt::legend({"vx", "vy", "vq"});
    plt::show(); // 显示图像(阻塞 并且显示所有图像)

}

TEST(NonTrajOptTest, CostGradDebug) {
    NonTrajOpt nontrajopt;
    EigenCSV eigen_csv;
    OptParas paras;
    paras.lambda_smo = 0.1;
    paras.lambda_obs = 0.1;
    paras.lambda_dyn = 0.1;
    paras.lambda_tim = 2000.0;
    paras.lambda_ova = 0.1;

    paras.wq = 0.8;
    paras.dist_th = 0.5;
    paras.discredist = 0.05;

    //// MATLAB 1.5*ref_vel
    paras.pv_max = 2.1;
    paras.pa_max = 2.1;
    paras.wv_max = 2.1;
    paras.wa_max = 2.1;
    paras.dyn_rate = 0.8;

    paras.ORIEN_VEL = 2.0;
    paras.VERDIT_VEL = 1.0;
    paras.OVAL_TH = 0.4;

    paras.SMO_SWITCH = true;
    paras.OBS_SWITCH = false;
    paras.DYN_SWITCH = false;
    paras.TIM_SWITCH = false;
    paras.OVA_SWITCH = false;

    paras.TIME_OPTIMIZATION = false;
    paras.REDUCE_ORDER = true;
    paras.BOUND_OPTIMIZATION = true;

    paras.INIT_OPT_VALUE  = true;

    paras.nlopt_max_iteration_num_ = 1000;
    paras.nlopt_max_iteration_time_ = 10.0;

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
    _initT << 1.2063, 0.3520, 0.6880;

    Eigen::Matrix<double, 3, 4> _startStates;
    _startStates.resize(3, 4);
    _startStates.setZero();
    _startStates.col(0) = _waypoints.col(0);

    Eigen::Matrix<double, 3, 4> _endStates;
    _endStates.resize(3, 4);
    _endStates.setZero();
    _endStates.col(0) = _waypoints.col(3);

    nontrajopt.initWaypoints(_waypoints, _initT, _startStates, _endStates);

    std::cout << "OptDof: " << nontrajopt.OptDof << std::endl;
    std::cout << "MatDim: " << nontrajopt.MatDim << std::endl;
    std::cout << "OptNum: " << nontrajopt.OptNum << std::endl;

    Eigen::VectorXd _optx;
    _optx.resize(nontrajopt.OptDof + 3);
    _optx.setZero();    
    _optx.tail(3) = _initT.array().log();
    Eigen::VectorXd _coeff;
    _coeff.resize(nontrajopt.MatDim);
    _coeff << 2, 0, 0, 0, 2.5996, -4.7631, 3.0521, -0.6598,
            4.5, 0, 0, 0, 0.1842, 0.5642, -0.625, 0.1577,
            1.5708, 0, 0, 0, -0.4839, -0.9201, 1.2507, -0.356,
            2.2902, 0.3699, 0.6392, 1.4873, 0.2336, -4.8276, 2.3715, 0.0449,
            4.9912, 1.088, 0.1168, -1.1587, -0.3682, 0.8588, 0.7065, -0.7962,
            0.7264, -1.65, 0.252, 1.7995, 0.4458, -10.3844, 17.0949, -7.4132,
            2.5465, 1.1204, 0.8292, -2.0726, -3.4322, 2.55, 7.6172, -6.397,
            5.3379, 0.7535, -0.9336, -0.4246, 1.241, 0.2793, -1.2553, 0.4926,
            0.2336, -1.0674, 1.05, 0.4888, 4.0399, -16.4143, 18.8061, -7.1919;
    Eigen::VectorXd optCoeffs = _coeff;


    // _coeff  << 2.0, 0.0,    0.0,    0.0,    2.5996, -4.7631, 3.0521, -0.6598,
    //         4.5,    0.0,    0.0,    0.0,    0.1842, 0.5642, -0.625, 0.1577,
    //         1.5708, 0.0,    0.0,    0.0,    -0.4839,-0.9201,1.2507, -0.356,
    //         2.2902, 0.3699, 0.6392, 1.4873, 0.2336, -4.8276,2.3715, 0.0449,
    //         4.9912, 1.088,  0.1168, -1.1587,-0.3682,0.8588, 0.7065, -0.7962,
    //         0.7264, -1.65,  0.252,  1.7995, 0.4458, -10.3844,17.0949, -7.4132,
    //         2.5465, 1.1204, 0.8292, -2.0726,-3.4322,2.55,   7.6172, -6.397,
    //         5.3379, 0.7535, -0.9336,-0.4246,1.241,  0.2793, -1.2553, 0.4926,
    //         0.2336, -1.0674,1.05,   0.4888, 4.0399, -16.4143, 18.8061, -7.1919;

    EXPECT_EQ(_coeff.size(), nontrajopt.MatDim);

    nontrajopt.getReduceOrder(_coeff);
    // std::cout << "reduce coeffs : " << _coeff.transpose() << std::endl; 
    // std::cout << "optIndex : " ;
    // for (int idx = 0;idx < nontrajopt.MatDim;idx++) {
    //     if (nontrajopt.optFlag[idx]) {
    //         std::cout << idx << " ";
    //     }
    // }
    // std::cout << std::endl;
    
    _optx.head(nontrajopt.OptDof) = _coeff;

    nontrajopt.updateOptVars(_optx);

    // 真奇怪 这里求解一个满秩的线性方程组 会出现求解结果不一致的情况
    // 必须得判断一下 是数值精度的问题还是自己的代码有问题
    nontrajopt.updateOptAxb();    // update MatA, Vecb by time initT

    Eigen::MatrixXd MatVec;
    MatVec.resize(nontrajopt.MatDim, nontrajopt.MatDim + 2);
    MatVec.setZero();
    MatVec.block(0, 0, nontrajopt.MatDim, nontrajopt.MatDim) = nontrajopt.MatA;
    MatVec.block(0, nontrajopt.MatDim, nontrajopt.MatDim, 1) = nontrajopt.Vecx;
    MatVec.block(0, nontrajopt.MatDim + 1, nontrajopt.MatDim, 1) = nontrajopt.Vecb;
    eigen_csv.FirstLineIsTitles = false;
    eigen_csv.WriteMatrix(MatVec, "/media/zwt/UbuntuFiles/datas/Swift/MatVec.csv");

    // 不使用线性方程组求得的解 给定coeffs 用于测试
    nontrajopt.Vecx = optCoeffs;

    nontrajopt.updateTraj();

    std::cout << "Vect :" << std::endl;
    std::cout << "T1 :" << nontrajopt.T1.transpose() << std::endl;
    std::cout << "T2 :" << nontrajopt.T2.transpose() << std::endl;
    std::cout << "T3 :" << nontrajopt.T3.transpose() << std::endl;
    std::cout << "T4 :" << nontrajopt.T4.transpose() << std::endl;
    std::cout << "T5 :" << nontrajopt.T5.transpose() << std::endl;
    std::cout << "T6 :" << nontrajopt.T6.transpose() << std::endl;
    std::cout << "T7 :" << nontrajopt.T7.transpose() << std::endl;

    std::cout << "Time " << nontrajopt.Traj.getDurations().transpose() << std::endl;
    std::cout << "Dist " << nontrajopt.discDist.transpose() << std::endl;
    std::cout << "Bars " << nontrajopt.Traj.getDiscreteNums().transpose() << std::endl;
    std::cout << "dt   " ;
    for (int idx = 0; idx < nontrajopt.N; idx++) {
        std::cout << nontrajopt.Traj[idx].getDiscretet() << " ";
    }
    std::cout <<  std::endl;

    Eigen::MatrixXd _statestraj;
    int state_num = nontrajopt.Traj.getTotalDiscreteNum();
    _statestraj.resize(state_num + 3, 12);
    int state_rows = 0;
    EXPECT_EQ(nontrajopt.Traj.getPieceNum(), 3);
    for (int idx = 0; idx < nontrajopt.Traj.getPieceNum(); idx++) {
        Eigen::MatrixXd _states;
        _states.resize(nontrajopt.Traj[idx].getDiscreteNum() + 1, 12);
        _states.col(0) = nontrajopt.Traj[idx].posVector.row(0).transpose();
        _states.col(1) = nontrajopt.Traj[idx].posVector.row(1).transpose();
        _states.col(2) = nontrajopt.Traj[idx].posVector.row(2).transpose();
        _states.col(3) = nontrajopt.Traj[idx].velVector.row(0).transpose();
        _states.col(4) = nontrajopt.Traj[idx].velVector.row(1).transpose();
        _states.col(5) = nontrajopt.Traj[idx].velVector.row(2).transpose();
        _states.col(6) = nontrajopt.Traj[idx].accVector.row(0).transpose();
        _states.col(7) = nontrajopt.Traj[idx].accVector.row(1).transpose();
        _states.col(8) = nontrajopt.Traj[idx].accVector.row(2).transpose();
        _states.col(9) = nontrajopt.Traj[idx].jerVector.row(0).transpose();
        _states.col(10) = nontrajopt.Traj[idx].jerVector.row(1).transpose();
        _states.col(11) = nontrajopt.Traj[idx].jerVector.row(2).transpose();
        _statestraj.block(state_rows, 0, _states.rows(), 12) = _states;
        state_rows += _states.rows();
    }
    eigen_csv.FirstLineIsTitles = true;
    std::string statetitle = "px, py, pq, vx, vy, vq, ax, ay, aq, jx, jy, jq";
    eigen_csv.setTitles(statetitle);
    eigen_csv.WriteMatrix(_statestraj, "/media/zwt/UbuntuFiles/datas/Swift/StatesTraj.csv");

    Eigen::MatrixXd TimeMat;
    TimeMat.resize(4*nontrajopt.N,nontrajopt.O);
    for (int idx = 0; idx < nontrajopt.N; idx++) {
        TimeMat.block(idx*4,0,4,nontrajopt.O) = nontrajopt.Traj[idx].timeMat;
    }
    eigen_csv.FirstLineIsTitles = false;
    eigen_csv.WriteMatrix(TimeMat, "/media/zwt/UbuntuFiles/datas/Swift/TimeMat.csv");


    nontrajopt.updateAeqbeq();
    Eigen::MatrixXd EquMatVec;
    EquMatVec.resize(nontrajopt.EquDim, nontrajopt.MatDim + 1);
    EquMatVec.setZero();
    EquMatVec.block(0, 0, nontrajopt.EquDim, nontrajopt.MatDim) = nontrajopt.MatAeq;
    EquMatVec.block(0, nontrajopt.MatDim, nontrajopt.EquDim, 1) = nontrajopt.Vecbeq;
    eigen_csv.FirstLineIsTitles = false;
    eigen_csv.WriteMatrix(EquMatVec, "/media/zwt/UbuntuFiles/datas/Swift/EquMatVec.csv");

    // 设置 EDF 的参数 map_width, map_height, map_resolution, mini_dist
    nontrajopt.setEDFMap(12,8,0.01,0.01);

    // 读取地图
    nontrajopt.readEDFMap("/home/zwt/motion_ws/src/navigation/grid_path_searcher/map/map10.png",0);

    std::cout << "Optimization Parameters: " << std::endl;
    std::cout << "pv_max: " << nontrajopt.pv_max << std::endl;
    std::cout << "pa_max: " << nontrajopt.pa_max << std::endl;
    std::cout << "wv_max: " << nontrajopt.wv_max << std::endl;
    std::cout << "wa_max: " << nontrajopt.wa_max << std::endl;
    std::cout << "wq: " << nontrajopt.wq << std::endl;
    std::cout << "dist_th: " << nontrajopt.dist_th << std::endl;
    std::cout << "discredist: " << nontrajopt.discredist << std::endl;
    std::cout << "oval_th: " << nontrajopt.OVAL_TH << std::endl;

    // nontrajopt.NLoptobjection(_optx, nullptr, nullptr);
    Eigen::VectorXd _grad;
    _grad.resize(nontrajopt.OptNum);

    double smoCost = 0;
    Eigen::VectorXd smoGradc = Eigen::VectorXd::Zero(nontrajopt.MatDim);
    Eigen::VectorXd smoGradt = Eigen::VectorXd::Zero(nontrajopt.N);
    nontrajopt.calcuSmoCost(smoCost, smoGradc, smoGradt);
    eigen_csv.FirstLineIsTitles = true;
    std::string gdsmotctitle = "seg1, seg2, seg3";
    eigen_csv.setTitles(gdsmotctitle);
    eigen_csv.WriteMatrix(nontrajopt.gdsmotc, "/media/zwt/UbuntuFiles/datas/Swift/gdsmotc.csv");
    std::cout << "gdsmocostgrad : " << std::endl;
    std::cout << nontrajopt.gdsmocostgrad << std::endl;
    // std::cout << "gdsmotc : " << std::endl;
    // std::cout << nontrajopt.gdsmotc << std::endl;

    double obsCost = 0;
    Eigen::VectorXd obsGradc = Eigen::VectorXd::Zero(nontrajopt.MatDim);
    Eigen::VectorXd obsGradt = Eigen::VectorXd::Zero(nontrajopt.N);
    nontrajopt.calcuObsCost(obsCost, obsGradc, obsGradt);
    eigen_csv.FirstLineIsTitles = true;
    std::string gdobstctitle = "xpos, ypos, dist, xgrad, ygrad, Tdt, cost, velnorm, 1/dist^2, -2*dist^3";
    eigen_csv.setTitles(gdobstctitle);
    eigen_csv.WriteMatrix(nontrajopt.gdobstc.transpose(), "/media/zwt/UbuntuFiles/datas/Swift/gdobstc.csv");
    std::string gdobstimetitle = "t^0, t^1, t^2, t^3, t^4, t^5, t^6, t^7";
    eigen_csv.setTitles(gdobstimetitle);
    eigen_csv.WriteMatrix(nontrajopt.gdobstimemat.transpose(), "/media/zwt/UbuntuFiles/datas/Swift/gdobstimet.csv");

    double dynCost = 0;
    Eigen::VectorXd dynGradc = Eigen::VectorXd::Zero(nontrajopt.MatDim);
    Eigen::VectorXd dynGradt = Eigen::VectorXd::Zero(nontrajopt.N);
    nontrajopt.calcuDynCost(dynCost, dynGradc, dynGradt);
    eigen_csv.FirstLineIsTitles = true;
    std::string gddyntctitle = "Tdt, deltaVel(0), 1, 2, deltaAcc(0), 1, 2, velCost, accCost, velgradt, accgradt, velnorm, accnorm, jernorm";
    eigen_csv.setTitles(gddyntctitle);
    eigen_csv.WriteMatrix(nontrajopt.gddyntc.transpose(), "/media/zwt/UbuntuFiles/datas/Swift/gddyntc.csv");

    double timCost = 0;
    Eigen::VectorXd timGradc = Eigen::VectorXd::Zero(nontrajopt.MatDim);
    Eigen::VectorXd timGradt = Eigen::VectorXd::Zero(nontrajopt.N);
    nontrajopt.calcuTimCost(timCost, timGradc, timGradt);

    double ovaCost = 0;
    Eigen::VectorXd ovaGradc = Eigen::VectorXd::Zero(nontrajopt.MatDim);
    Eigen::VectorXd ovaGradt = Eigen::VectorXd::Zero(nontrajopt.N);
    nontrajopt.calcuOvaCost(ovaCost, ovaGradc, ovaGradt);
    eigen_csv.FirstLineIsTitles = true;
    std::string gdovatctitle = "Tdt, vel_B(1), vel_B(2), vel_B2, R(1v1), R(1v2), R(2v1), R(2v2), gradt(idx),\
                                xgrad, ygrad, qgrad, acc_B(1), acc_B(2), acc_B2, yaw_q";
    eigen_csv.setTitles(gdovatctitle);
    eigen_csv.WriteMatrix(nontrajopt.gdovatc.transpose(), "/media/zwt/UbuntuFiles/datas/Swift/gdovatc.csv");

    double Cost = 0.0;
    Cost =  smoCost * nontrajopt.lambda_smo + 
            obsCost * nontrajopt.lambda_obs + 
            dynCost * nontrajopt.lambda_dyn + 
            timCost * nontrajopt.lambda_tim + 
            ovaCost * nontrajopt.lambda_ova;

    eigen_csv.FirstLineIsTitles = true;
    std::string costtitle = "smoCost, obsCost, dynCost, timCost, ovaCost";
    eigen_csv.setTitles(costtitle);
    Eigen::MatrixXd Gradc = Eigen::MatrixXd::Zero(nontrajopt.MatDim, 5);
    Eigen::MatrixXd Gradt = Eigen::MatrixXd::Zero(nontrajopt.N, 5);
    Gradc.col(0) = smoGradc;
    Gradc.col(1) = obsGradc;
    Gradc.col(2) = dynGradc;
    Gradc.col(3) = timGradc;
    Gradc.col(4) = ovaGradc;
    Gradt.col(0) = smoGradt;
    Gradt.col(1) = obsGradt;
    Gradt.col(2) = dynGradt;
    Gradt.col(3) = timGradt;
    Gradt.col(4) = ovaGradt;
    eigen_csv.WriteMatrix(Gradc, "/media/zwt/UbuntuFiles/datas/Swift/Gradc.csv");
    eigen_csv.WriteMatrix(Gradt, "/media/zwt/UbuntuFiles/datas/Swift/Gradt.csv");
    std::cout << "AllCost: " << Cost << " ";
    std::cout << " smoCost: " << smoCost;
    std::cout << " obsCost: " << obsCost;
    std::cout << " dynCost: " << dynCost;
    std::cout << " timCost: " << timCost;
    std::cout << " ovaCost: " << ovaCost << std::endl;

    // nontrajopt.getReduceOrder(smoGradc);
    // smoGradc *= nontrajopt.lambda_smo;
    // smoGradt *= nontrajopt.lambda_smo;
    // _grad.head(nontrajopt.OptDof) += smoGradc;
    // if (nontrajopt.TIME_OPTIMIZATION)
    //     _grad.tail(nontrajopt.N) += smoGradt;
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
    _initT << 1.2063, 0.3520, 0.6880;

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

    // eigen_csv.WriteMatrix(nontrajopt.MatQ, "/media/zwt/UbuntuFiles/datas/Swift/MatQ.csv");
    // std::cout << "nontrajopt.MatQ: rank = "<< nontrajopt.MatQ.fullPivLu().rank() << std::endl;
    // Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigensolver(nontrajopt.MatQ);
    // std::cout << "SelfAdjointEigenSolver : minicoeffs = " << eigensolver.eigenvalues().minCoeff() << std::endl;
    // eigen_csv.WriteMatrix(nontrajopt.MatAeq, "/media/zwt/UbuntuFiles/datas/Swift/MatAeq.csv");
    // std::cout << "nontrajopt.MatAeq: rank = " << nontrajopt.MatAeq.fullPivLu().rank() <<  std::endl;
    // eigen_csv.WriteVector(nontrajopt.Vecbeq, "/media/zwt/UbuntuFiles/datas/Swift/Vecbeq.csv");
    // std::cout << "nontrajopt.Vecbeq: " << std::endl;

    nontrajopt.OSQPSolve();
}