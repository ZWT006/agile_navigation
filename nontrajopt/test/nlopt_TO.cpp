/*
 * @Author: wentao zhang && zwt190315@163.com
 * @Date: 2023-07-20
 * @LastEditTime: 2023-07-21
 * @Description: 
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

// 绘图函数
namespace plt = matplotlibcpp;
// .csv 文件读写
EigenCSV eigen_csv;

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);

    // testing::GTEST_FLAG(filter) = "NonTrajOptTest.NonlinearSolver";
    testing::GTEST_FLAG(filter) = "NonTrajOptTest.NonlinearLongSolver";
    return RUN_ALL_TESTS();
}



TEST(NonTrajOptTest, NonlinearSolver) {
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
    nontrajopt.reset(18);

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

    
    nontrajopt.updateAeqbeq();    // update MatA, Vecb by time initT  
    nontrajopt.updateMatQ();      // update MatQ by time initT
    nontrajopt.OSQPSolve();       // solve QP problem
    // nontrajopt.OSQP_optx;         // get opt result

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
    std::cout << "MaxIterTime: " << std::fixed << std::setprecision(2) << nontrajopt.nlopt_max_iteration_time_ << std::endl;

    bool succeed = nontrajopt.NLoptSolve();
    // nontrajopt.Non_optx;    // get opt result
    nontrajopt.updateOptVars(nontrajopt.Non_optx);
    // 更新优化变量对应的矩阵 并求解 Ax=b
    nontrajopt.updateOptAxb();
    // 将求解后的系数放入 Traj 中
    nontrajopt.updateTraj();    // 得到最终的状态轨迹

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

TEST(NonTrajOptTest, NonlinearLongSolver) {
    NonTrajOpt nontrajopt;
    EigenCSV eigen_csv;
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

    paras.ref_vel  = 1.4;
    paras.ref_ome  = 1.4;

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

    paras.nlopt_max_iteration_num_ = 8000;
    paras.nlopt_max_iteration_time_ = 1000.0;

    nontrajopt.initParameter(paras);

    int pieceNum = 18;

    nontrajopt.reset(pieceNum);

    Eigen::Matrix3Xd _waypoints;
    _waypoints.resize(3, pieceNum + 1);
    // 输入数据会按列的顺序进行排列
    _waypoints <<   0.5, 0.707167804, 1.261955909, 1.896318877, 2.872191732, 3.731390967, 4.567990136, 5.197474604, 5.50910145, 5.616630397, 6.059238311, 6.964136867, 7.744251763, 8.018312017, 8.757446927, 9.717516773, 10.64704588, 11.5468593, 11.53,
        0.5, 0.533438733, 0.765041719, 1.054227166, 1.571125841, 2.189972565, 2.256812825, 3.108813267, 3.998833057, 4.379295486, 4.610732276, 4.666486593, 4.020906861, 3.724072537, 3.104587759, 2.906035106, 2.859908021, 2.322542144, 1.45,
        0.785398163, 0.27774869, 0.41159563, 0.457414996, 0.555650785, 0.351960015, 0.50710377, 1.084241581, 1.264677385, 0.888573747, 0.271665149, -0.314888017, -0.758290526, -0.761413146, -0.450746478, -0.126759527, -0.293969696, 0.12352112, 0.785398163;

    // std::cout << "_waypoints: " << _waypoints << std::endl;

    Eigen::VectorXd _initTexp,_initT;
    _initTexp.resize(pieceNum);
    _initTexp << -0.321289138,-0.845315982,
        -0.697199683,-0.23724682,
        -0.279274877,-0.511700972,
        -0.278846565,-0.395162628,
        -1.264416044,
        -0.819506642,
        -0.43451013,-0.323954037,
        -1.242801491,-0.372713604,
        -0.356280855,-0.408319628,
        -0.289532987,0.220517256;
    _initT.resize(pieceNum);
    _initT = _initTexp.array().exp();
    std::cout << "_initT: " << std::endl;
    std::cout << _initT.transpose() << std::endl;

    Eigen::Matrix<double, 3, 4> _startStates;
    _startStates.resize(3, 4);
    _startStates.setZero();
    _startStates.col(0) = _waypoints.col(0);

    Eigen::Matrix<double, 3, 4> _endStates;
    _endStates.resize(3, 4);
    _endStates.setZero();
    _endStates.col(0) = _waypoints.col(pieceNum);

    nontrajopt.initWaypoints(_waypoints, _initT, _startStates, _endStates);

    // double before_opt_time = _initT.sum();

    std::cout << "OptDof: " << nontrajopt.OptDof << std::endl;
    std::cout << "MatDim: " << nontrajopt.MatDim << std::endl;
    std::cout << "OptNum: " << nontrajopt.OptNum << std::endl;

    Eigen::VectorXd _optx;
    _optx.resize(nontrajopt.OptDof + pieceNum);
    _optx.setZero();    
    _optx.tail(pieceNum) = _initT.array().log();
    Eigen::VectorXd _coeff;
    _coeff.resize(nontrajopt.MatDim);
    
    eigen_csv.FirstLineIsTitles = false;
    std::string _coeff_path = "/media/zwt/UbuntuFiles/datas/Swift/QPcoeffs.csv";
    eigen_csv.ReadVector(_coeff,_coeff_path);

    Eigen::VectorXd new_coeff = _coeff.head(pieceNum * 24);

    _coeff = new_coeff;


    nontrajopt.updateAeqbeq();    // update MatA, Vecb by time initT  
    nontrajopt.updateMatQ();      // update MatQ by time initT
    nontrajopt.OSQPSolve();       // solve QP problem
    nontrajopt.updateTraj();

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

    plt::figure_size(1200, 800);
    plt::plot(px, py,"g.");
    plt::title("[x.y] position figure");

    plt::figure_size(1200, 800);
    // Plot line from given x and y data. Color is selected automatically.
    plt::plot(time, pq,"b-");
    // Add graph title
    plt::title("[q] position figure");

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

    _coeff = nontrajopt.OSQP_optx;

    EXPECT_EQ(_coeff.size(), nontrajopt.MatDim);

    nontrajopt.getReduceOrder(_coeff);
    
    _optx.head(nontrajopt.OptDof) = _coeff;

    

    // nontrajopt.OSQP_optx;         // get opt result

    nontrajopt.updateOptVars(_optx);
    nontrajopt.updateOptAxb();    // update MatA, Vecb by time initT
    nontrajopt.updateTraj();

    

    double before_opt_time = nontrajopt.Traj.getTotalDuration();

    // 设置 EDF 的参数 map_width, map_height, map_resolution, mini_dist
    nontrajopt.setEDFMap(12,8,0.01,0.01);

    // 读取地图
    nontrajopt.readEDFMap("/home/zwt/motion_ws/src/navigation/grid_path_searcher/map/map5.png",0);

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
    std::cout << "MaxIterTime: " << std::fixed << std::setprecision(2) << nontrajopt.nlopt_max_iteration_time_ << std::endl;

    bool succeed = nontrajopt.NLoptSolve();
    // nontrajopt.Non_optx;    // get opt result
    nontrajopt.updateOptVars(nontrajopt.Non_optx);
    // 更新优化变量对应的矩阵 并求解 Ax=b
    nontrajopt.updateOptAxb();
    // 将求解后的系数放入 Traj 中
    nontrajopt.updateTraj();    // 得到最终的状态轨迹

    double after_opt_time =nontrajopt.Traj.getTotalDuration();

    std::cout << "before_opt_time: " << before_opt_time << "; after_opt_time: " << after_opt_time << std::endl;

    if (succeed)
        std::cout << GREEN << "NLoptSolve succeed! Happy!!!" << RESET << std::endl;
    else
        std::cout << RED << "NLoptSolve failed! Sad!!!" << RESET << std::endl;

    // eigen_csv.WriteMatrix(_traj, "/home/zwt/Documents/TRAJ_STATE.csv");

    dt = 0.01;
    allT = nontrajopt.Traj.getTotalDuration();
    num = allT / dt;
    time.resize(num);
    px.resize(num); py.resize(num); pq.resize(num);
    vx.resize(num); vy.resize(num); vq.resize(num);
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

    ////TODO: 感觉这里可能还有问题 plot 出来的图像 y轴数值非常大 感觉不科学呀
    plt::figure_size(1200, 800);
    plt::plot(px, py,"g.");
    plt::title("[x.y] position figure");


    plt::figure_size(1200, 800);
    // Plot line from given x and y data. Color is selected automatically.
    plt::plot(time, pq,"b-");
    // Add graph title
    plt::title("[q] position figure");

    


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
// 0.5, 0.707167804, 1.261955909, 1.896318877, 2.872191732, 3.731390967, 4.567990136, 5.197474604, 5.50910145, 5.616630397, 6.059238311, 6.964136867, 7.744251763, 8.018312017, 8.757446927, 9.717516773, 10.64704588, 11.5468593, 11.53,
// 0.5, 0.533438733, 0.765041719, 1.054227166, 1.571125841, 2.189972565, 2.256812825, 3.108813267, 3.998833057, 4.379295486, 4.610732276, 4.666486593, 4.020906861, 3.724072537, 3.104587759, 2.906035106, 2.859908021, 2.322542144, 1.45,
// 0.785398163, 0.27774869, 0.41159563, 0.457414996, 0.555650785, 0.351960015, 0.50710377, 1.084241581, 1.264677385, 0.888573747, 0.271665149, -0.314888017, -0.758290526, -0.761413146, -0.450746478, -0.126759527, -0.293969696, 0.12352112, 0.785398163;
