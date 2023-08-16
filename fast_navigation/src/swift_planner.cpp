/*
 * @Author: wentao zhang && zwt190315@163.com
 * @Date: 2023-06-23
 * @LastEditTime: 2023-08-16
 * @Description: swaft planner for fast real time navigation 
 * @reference: 
 * 
 * Copyright (c) 2023 by wentao zhang, All Rights Reserved. 
 */

#include <Eigen/Geometry>
// lazykinoprm path searching
#include <lazykinoprm/LazyKinoPRM.h>
// nontrajopt trajectory optimization
#include <nontrajopt/nontrajopt.h>
#include <fast_navigation/navigation.hpp>
#include <fast_navigation/readwritecsv.hpp>
// // for visualization
// #include "matplotlibcpp.h"
// // 绘图函数
// namespace plt = matplotlibcpp;

using namespace std;
using namespace Eigen;
using namespace cv;

// robot visulaization param
//unitree A1 robot size
#define DEFAULT_HIGH 0.30f
#define DEFAULT_WIDTH 0.40f
#define DEFAULT_LENGTH 0.50f

#define Q_WEIGHT 0.01f

// visualization_msgs::Marker 可视化的透明度
#define RGB_ALPHA 0.6f

// ros related global variables
ros::Subscriber _map_sub, _pts_sub, _odom_sub;  // 订阅地图;导航点;里程计
ros::Subscriber _goal_sub;                      // 订阅目标点
int _map_sub_cnt = 5, _odom_sub_cnt = 5;        // 计数器 判断是否正常sub
#define SUB_CNT_TH 5                           // 订阅计数器阈值 Hz = 10
#define REPLAN_CNT_TH 5                         // 重规划计数器阈值
#define SEARCH_CNT_TH 10                         // 搜索计数器阈值

// ros::Publisher _nav_seq_vis_pub;    // 导航点可视化
ros::Publisher _obsmap_img_pub,_sdfmap_img_pub; // 地图可视化
ros::Publisher _ptraj_vis_pub,_vtraj_vis_pub;   // 轨迹可视化
ros::Publisher _path_vis_pub,_robot_vis_pub;    // 导航可视化
ros::Publisher _osqp_vis_pub,_nlopt_vis_pub;    // 优化轨迹可视化
ros::Publisher _track_vis_pub,_real_vis_pub;    // 跟踪轨迹可视化

ros::Publisher _obs_map_pub;    // 全局障碍物地图
// 各部分计算的计时 帮助调试,分析代码
ros::Time _time_opt_start;                // ros 时间
ros::Time _time_opt_end;                  // ros 时间
ros::Time _time_search_start;             // ros 时间
ros::Time _time_search_end;               // ros 时间

////####################################################################################
// node parameters
double _local_width;    // local map width
double _loop_rate;      // node loop rate
double _time_interval;  // time interval for trajectory
// visualization switch
bool _vis_Robot = false;
bool _vis_sample = false;
bool _vis_traj = false;
bool _vis_obs_pcl = true;
bool _vis_obs_img = false;
bool _vis_sdf_img = false;
bool _nav_seq_vis_flag = true;
bool _vis_osqp_traj = false;
bool _vis_nlopt_traj = false;
bool _vis_tracking_traj = false;
bool _vis_real_traj = false;
double _vis_resolution = 0.02; // 可视化的分辨率
double _resolution;     // obs map 分辨率
// 轨迹安全性检查长度分辨率
double _DIST_RES = 0.04; // 4cm

////####################################################################################
// trajectory tracking GLOBAL FLAGS
bool _HAS_MAP   = false;    // 是否有地图
bool _LOCAL_PCL = false;    // 是是局部点云地图 yes 就是局部点云地图 no 就是全局点云地图
bool _HAS_ODOM  = false;    // 是否有里程计信息
bool _HAS_PATH  = false;    // 搜索到路径，每次 lazykinoprm 搜索到路径后，就会置为true; Obstacle check infeasible 会置为false
bool _NEW_PATH  = true;     // 是否是新的路径 重新设置目标点后,就是新的路径
bool _FIRST_ODOM= true;     // 是否是第一次接收到里程计信息 
bool _REACH_GOAL= true;    // 是否到达目标点
bool _TRACKING  = false;    // 是否正在跟踪轨迹 安全检查不通过就会置为false 原地急刹车trot

bool _OFFESET_ODOM = false; // 是否需要反向里程计信息
bool _PURE_TRACKING = false; // 是否纯跟踪模式
bool _DEBUG_REPLAN = false; // 是否 debug replan 模式
bool _OPT_TRAJ = false; // 是否优化轨迹

////####################################################################################
// trajectory optimization parameters 
int _OPT_SEG = 0;           // 优化轨迹的段数 这个参数根据 sample 的 grid size 和 reference velocity 来确定 
                            // 小于0就不优化 | 等于0就是全优化 | 大于0就是根据采样区间和参考速度计算优化长度
double _optHorizon = 1.5;   // 优化轨迹的时间长度
#define OPT_FACTOR 1.5f
// nav_msgs::Odometry currodometry_msg;
nav_msgs::Odometry::Ptr currodometry;
// 感觉没啥用
// Eigen::Isometry3d Odomtrans; // 坐标系转换
// double biasx = 0.0, biasy = 0.0,biasq = 0.0; // 坐标系转换偏置 [x,y,q] q:rad
// double cosyaw,sinyaw; // 坐标系转换偏置 [x,y,q] q:rad
// visualization robot switch 在ros循环中可视化 
Vector3d _first_pose;

// Trajectory Read and Write
ReadCSV readcsv;

// Trajectory Tracking Manager
Tracking tracking;
// Lazy Kinodynamic Path Searching
LazyKinoPRM lazykinoPRM;
// Trajectory Optimization
NonTrajOpt OSQPopt;
NonTrajOpt nloptopt;
Eigen::VectorXd _opt_init;
// nonlinear optimization parameters
OptParas paras;

// Trajectory Struct
vector<TrackSeg> searchTraj;    // 存储搜索到的轨迹
vector<TrackSeg> optimalTraj;   // 存储优化的轨迹

// 回调函数 ############################################################################################################
// 目标点回调函数，进行路径规划
void rcvWaypointsCallback(const nav_msgs::Path & wp);
// 点云回调函数，负责设置障碍地图并更新SDF地图
void rcvPointCloudCallback(const sensor_msgs::PointCloud2 & pointcloud_map);
// 里程计回调函数，负责设置机器人位置;进行轨迹跟踪
void rcvOdomCallback(const nav_msgs::Odometry::Ptr& msg);

// 关键功能函数 ############################################################################################################
// search trajectory checking
bool TrackTrajCheck();
// 搜索到的 tracksegs 从 lazykinoprm 类 push 到 searchTraj 列表
bool SearchSegPush();
bool OSQPSegPush();
// 待优化的 tracksegs 从 Tracking tracking 类 pop 到 optimalTraj 列表 
bool OptimalSegPop();
// 优化后的 tracksegs 从 NonTrajOpt nloptopt 类 push 到 optimalTraj 列表中
void OptimalSegPush();
// 轨迹跟踪的安全性检查
bool NavSeqCheck(const nav_msgs::Path *navseq);

// 可视化函数 ############################################################################################################
// visulization search trajectory
void visTrackingTraj();
void visRealTraj();
void visLazyPRM();
void visOSQPTraj();
void visNLOptTraj();
// visulization tracking trajectory
// bool visTrackingTraj();
// visulization obstacle map
void visObsMap();
// visulization robot 
void visRobot(const nav_msgs::Odometry::Ptr& msg);
// visulization position trajectory
void visPtraj();
// visulization velocity trajectory
void visVtraj();
// save trajectory as .csv file
void csvPtraj();

int main(int argc, char** argv)
{
    ros::init(argc, argv, "swift_planner_node");
    ros::NodeHandle nh("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);

    // 地图和导航点订阅
    _map_sub  = nh.subscribe( "map",       1, rcvPointCloudCallback );
    _pts_sub  = nh.subscribe( "waypoints", 1, rcvWaypointsCallback );
    _odom_sub = nh.subscribe( "odom",      1, rcvOdomCallback );
    _goal_sub = nh.subscribe( "goal",      1, rcvWaypointsCallback );

    // 路径和地图可视化
    _path_vis_pub       = nh.advertise<visualization_msgs::MarkerArray>("planner_path_vis",1);
    _osqp_vis_pub       = nh.advertise<visualization_msgs::MarkerArray>("planner_osqp_vis",1);
    _nlopt_vis_pub      = nh.advertise<visualization_msgs::MarkerArray>("planner_nlopt_vis",1);
    _track_vis_pub      = nh.advertise<visualization_msgs::MarkerArray>("planner_track_vis",1);
    _real_vis_pub       = nh.advertise<visualization_msgs::MarkerArray>("planner_real_vis",1);
    _obsmap_img_pub     = nh.advertise<sensor_msgs::Image>("obs_map_img",1);
    _sdfmap_img_pub     = nh.advertise<sensor_msgs::Image>("sdf_map_img",1);
    _robot_vis_pub      = nh.advertise<visualization_msgs::MarkerArray>("robot_vis",1);
    // _cmd_vel_pub              = nh.advertise<geometry_msgs::Twist>("/cmd_vel",1);
    // _nav_seq_vis_pub          = nh.advertise<nav_msgs::Path>("/nav_seq_vis",1);
    _obs_map_pub        = nh.advertise<sensor_msgs::PointCloud2>("obs_map_pcl",1);

    // 轨迹可视化
    _ptraj_vis_pub      = nh.advertise<geometry_msgs::PoseArray>("ptraj_vis",1);
    _vtraj_vis_pub      = nh.advertise<geometry_msgs::PoseArray>("vtraj_vis",1);

    // 关键功能类初始化 ############################################################################################################
    ROS_DEBUG("tracking.setParam(nh)");
    // Trajectory Tracking 使用参数
    tracking.setParam(nh);
    ROS_DEBUG("lazykinoPRM.setParam(nh)");
    // LazyKinoPRM 启动参数
    lazykinoPRM.setParam(nh);
    // 初始化相关参数
    ROS_DEBUG("lazykinoPRM.init()");
    lazykinoPRM.init();
    // 读取优化参数
    ROS_DEBUG("OSQPopt.setParam(nh,paras);");
    OSQPopt.setParam(nh,paras);
    // 初始化优化参数
    ROS_DEBUG("OSQPopt.initParameter(paras)");
    OSQPopt.initParameter(paras);
    nloptopt.initParameter(paras);
    // 参数打印
    ROS_DEBUG("OSQPopt.showParam()");
    OSQPopt.showParam();

    // 读取参数 ################################################################################################################
    // 地图相关参数
    nh.param("map/local_pcl",_LOCAL_PCL,true);
    nh.param("map/local_width", _local_width, 2.0);
    nh.param("map/resolution", _resolution, 0.05); // obs_map grid 5cm

    nh.param("search/time_interval",_time_interval,0.01);
    // 轨迹跟踪相关参数
    nh.param("planner/loop_rate", _loop_rate, 1.0);
    nh.param("planner/vis_Robot", _vis_Robot, false);
    nh.param("planner/vis_sample", _vis_sample, false);
    nh.param("planner/vis_traj", _vis_traj, false);
    nh.param("planner/vis_sdf_img", _vis_sdf_img, false);
    nh.param("planner/vis_obs_pcl", _vis_obs_pcl, true);
    nh.param("planner/vis_obs_img", _vis_obs_img, true);
    nh.param("planner/nav_seq_vis", _nav_seq_vis_flag, true);
    nh.param("planner/vis_osqp_traj", _vis_osqp_traj, false);
    nh.param("planner/vis_nlopt_traj", _vis_nlopt_traj, false);
    nh.param("planner/vis_tracking_traj", _vis_tracking_traj, false);
    nh.param("planner/vis_real_traj", _vis_real_traj, false);
    // nh.param("planner/time_interval", _time_interval, 0.1); 
    nh.param("planner/optHorizon", _optHorizon, 1.5); 
    // 直接根据 loop_rate 计算每次跟踪的时间间隔
    // _time_interval = double( 1 / _loop_rate);
    if (_DIST_RES > _resolution)
        _DIST_RES = _resolution * 0.8; 
    // 计算优化相关的参数 ################################################################################################################
    double _opt_seg = 0,xy_sample_size_ = 0;
    nh.param("planner/opt_seg", _opt_seg, 0.0);
    nh.param("planner/opt_traj",_OPT_TRAJ,false);
    nh.param("search/xy_sample_size", xy_sample_size_, 0.4);
    _OPT_SEG = int(_opt_seg);
    // if (_OPT_SEG > 0){ // 根据采样区间和参考速度计算优化长度
        // double sample_grid_time = xy_sample_size_ / tracking._refer_vel;
        // std::cout << "sample_grid_time: " << sample_grid_time << " xy_sample_size: " << xy_sample_size_ << " refer_vel: " << tracking._refer_vel << std::endl;
        // _OPT_SEG = int(std::ceil(_optHorizon / sample_grid_time * OPT_FACTOR)); // 优化轨迹的段数
        // std::cout << "opt_seg: " << _OPT_SEG << " opt" << std::endl;

    // }
    tracking._TO_SEG = _OPT_SEG;
    nh.param("planner/offset_odom", _OFFESET_ODOM, false);
    // 读取给定轨迹并跟踪 ################################################################################################################
    nh.param("planner/pure_tracking", _PURE_TRACKING, false);
    nh.param("planner/debug_replan", _DEBUG_REPLAN, false);
    std::string traj_address;
    nh.param("planner/traj_address",traj_address,std::string("datas/TRAJ_DATA_LONG.csv"));
    // FileName: TRAJ_DATA_LONG.csv | TRAJ_DATA_SHORT.csv
    if (_PURE_TRACKING) {
        // nh.param("planner/orign_biasx",biasx,0.0);
        // nh.param("planner/orign_biasy",biasy,0.0);
        // nh.param("planner/orign_biasq",biasq,0.0);
        readcsv.setFileName(traj_address);
        double traj_res = 0.01;
        nh.param("planner/traj_ration",traj_res,0.01);
        readcsv.setOrign(traj_res);
        TrackSeg _trackseg;
        cout << BLUE << "====================================================================" << RESET << endl;
        ROS_INFO("[\033[32mPlanNode\033[0m]: read trajectory from =>%s",traj_address.c_str());
        if (readcsv.readFile(_trackseg.pxtraj,_trackseg.pytraj,_trackseg.pqtraj,_trackseg.vxtraj,_trackseg.vytraj,_trackseg.vqtraj))
        {
            ROS_INFO("[\033[32mPlanNode\033[0m]: read trajectory success, size: %ld",_trackseg.pxtraj.size());
            _trackseg.duration = _trackseg.pxtraj.size() * _time_interval;
            Eigen::Vector3d goalPose; 
            goalPose << _trackseg.pxtraj.back(),_trackseg.pytraj.back(),_trackseg.pqtraj.back();
            tracking.initPlanner(goalPose);
            std::vector<TrackSeg> _tracksegsets;
            _tracksegsets.push_back(_trackseg);
            tracking.insertSegTraj(0,&_tracksegsets);
            tracking._goalPose = goalPose;
            ROS_INFO("[\033[32mPlanNode\033[0m]: goalPose: [%2.4f,%2.4f,%2.4f]",tracking._goalPose(0),tracking._goalPose(1),tracking._goalPose(2));
            if (_vis_tracking_traj) visTrackingTraj();
        }
        else ROS_INFO("[\033[31mPlanNode\033[0m]: read trajectory failed");
        cout << BLUE << "====================================================================" << RESET << endl;
    }
    
    // 坐标系转换相关参数 ################################################################################################################
    // Eigen::Vector3d _pose, _euler;
    // Eigen::Quaterniond _quat;
    // nh.param("planner/frame_x", _pose(0), 0.0);
    // nh.param("planner/frame_y", _pose(1), 0.0);
    // nh.param("planner/frame_z", _pose(2), 0.0);
    // nh.param("planner/frame_roll", _euler(0), 0.0);
    // nh.param("planner/frame_pitch", _euler(1), 0.0);
    // nh.param("planner/frame_yaw", _euler(2), 0.0);
    // tf::Quaternion quaternion;
    // quaternion.setRPY(_euler(0), _euler(1), _euler(2));
    // _quat.x() = quaternion.x(); _quat.y() = quaternion.y(); _quat.z() = quaternion.z(); _quat.w() = quaternion.w();
    // Odomtrans = Eigen::Isometry3d(_quat); // 将四元数转换为旋转矩阵
    // Odomtrans.pretranslate(_pose);
    // 可能没用 咋感觉这么麻烦呢? 学习使用坐标系转换是非常有必要的 但是这里的坐标系转化不如直接用tf 启动 SLAM 的时候加个偏置就行了
    // ####################################################################################

    // 轨迹跟踪相关参数
    ROS_INFO("[\033[34mPlanNode\033[0m]_local_width : %2.4f",    _local_width);
    ROS_INFO("[\033[34mPlanNode\033[0m]_resolution  : %2.4f",    _resolution);
    ROS_INFO("[\033[34mPlanNode\033[0m]_loop_rate   : %2.4f",    _loop_rate);
    ROS_INFO("[\033[34mPlanNode\033[0m]_time_interval: %2.4f",  _time_interval);
    ROS_INFO("[\033[34mPlanNode\033[0m]_optHorizon  : %2.4f",   _optHorizon);
    ROS_INFO("[\033[34mPlanNode\033[0m]_opt_seg     : %6d",     _OPT_SEG);
    ROS_INFO("[\033[34mPlanNode\033[0m]_REACH_GOAL  : %s",     _REACH_GOAL  ? "true" : "false");
    ROS_INFO("[\033[34mPlanNode\033[0m]_OPT_TRAJ    : %s",     _OPT_TRAJ ? "true" : "false");
    // ROS_INFO("[\033[34mPlanNode\033[0m]planner/time_horizon: %2.4f",        tracking._time_horizon);
    // ROS_INFO("[\033[34mPlanNode\033[0m]planner/traj_time_interval: %2.4f",  tracking._traj_time_interval);
    // ROS_INFO("[\033[34mPlanNode\033[0m]planner/time_step_interval: %d",     tracking._time_step_interval);
    // ROS_INFO("[\033[34mPlanNode\033[0m]planner/time_step_pursuit: %d",      tracking._time_step_pursuit);
    // ROS_INFO("[\033[34mPlanNode\033[0m]planner/vel_factor: %2.4f",          tracking._vel_factor);

    ros::Rate rate(_loop_rate);
    bool status = ros::ok();
    while(status) 
    {
        // step callbackfunction ############################################################################################################
        // step.1 waypoint callback     set goal pose
        // step.2 pointcloud callback   update obs map
        // step.3 odom callback         track trajectory
        ros::spinOnce();
        // _map_sub_cnt++;
        // if (_map_sub_cnt > SUB_CNT_TH) {
        //     // _HAS_MAP = false;
        //     ROS_INFO("[\033[31mPlanNode\033[0m]: !!! don't receive map !!!");
        //     _map_sub_cnt = 5;
        // }
        // _odom_sub_cnt++;
        // if (_odom_sub_cnt > SUB_CNT_TH) {
        //     // _HAS_ODOM = false;
        //     ROS_INFO("[\033[31mPlanNode\033[0m]: !!! don't receive odom !!!");
        //     _odom_sub_cnt = 5;
        // }
        
        // &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
        // 如果纯跟踪预定义轨迹,就不需要规划了 直接跳过规划 &
        if (_PURE_TRACKING || _DEBUG_REPLAN) goto JMUP_REPLAM; //&&&&&
        // &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
        
        //// step 轨迹的安全性检查 ############################################################################################################
        //// 检查接下来需要跟踪的轨迹是否安全 跟踪偏差大不进行重规划 只预定轨迹有障碍时才会重规划
        if (!_REACH_GOAL){
            bool feasible = TrackTrajCheck();
            // step.1 判断是否距离障碍物太近 就得急刹车了 这个舍弃了 太近了就直接急刹车了 停止规划重启算了
            if (tracking.TROT_FLAG){ // 障碍物距离太近,需要急刹车再重新规划
                _TRACKING = false;
                // tracking.NavSeqFixed(tracking._local_odom);
                // tracking.NavSeqPublish();
                ROS_INFO("[\033[31mPlanNode\033[0m]: too close to obstacle, braking!!!");
                ROS_INFO("[\033[31mPlanNode\033[0m]:currIndex:%d,obsIndex:%d,searchIndex:%d",tracking._current_index[0],tracking._obs_seg_index,tracking._search_seg_index);
                ROS_INFO("[\033[31mPlanNode\033[0m]: 放弃治疗 planner 就地摆烂");
                return 0;
            }
            // step.2 轨迹上有障碍物,正常重规划
            if (!feasible){  // 轨迹上有障碍物
                ROS_INFO("[\033[33mPlanNode\033[0m]: track trajectory is not feasible");
                _HAS_PATH = false;
                Vector3d goalPose,currPose,currVel,currAcc,zeros_pt(0,0,0);
                int re_plan_cnt = 0;
                while (_HAS_PATH == false && re_plan_cnt < REPLAN_CNT_TH){
                    re_plan_cnt++;
                    // step 重新规划轨迹 ####################
                    if(tracking.getReplanState(&currPose,&currVel,&currAcc,&goalPose)){
                        ros::Time start_time = ros::Time::now();
                        lazykinoPRM.reset();
                        _HAS_PATH = !lazykinoPRM.search(currPose,currVel,currAcc,goalPose,zeros_pt,zeros_pt);
                        ros::Duration elapsed_time = ros::Time::now() - start_time;
                        double elapsed_seconds = elapsed_time.toSec();
                        ROS_INFO("[\033[34mPlanNode\033[0m]: search time: %2.4f", elapsed_seconds);
                    }
                    else {
                        // lazykinoPRM.reset();
                        lazykinoPRM.sample();
                        ROS_INFO("[\033[31mPlanNode\033[0m]: get replan state failed");
                        continue;
                    }
                    if (_HAS_PATH){ // 重新规划成功
                        tracking._goalPose = lazykinoPRM._goal_pose;
                        if (_OPT_TRAJ) {
                            if(!OSQPSegPush()) {
                                _HAS_PATH = false;
                                lazykinoPRM.sample();
                                continue;
                            }
                        }
                        else
                            SearchSegPush();
                        
                        ROS_INFO("[\033[32mPlanNode\033[0m]searchindex:%d,search traj size:%ld",tracking._search_seg_index,searchTraj.size());
                        tracking.insertSegTraj(tracking._search_seg_index,&searchTraj);
                        ROS_INFO("[\033[32mPlanNode\033[0m]: replan success");
                        tracking.OBS_FLAG = false;
                        _TRACKING = true;
                        visLazyPRM(); // 可视化搜索轨迹 感觉就是DEBUG的时候用
                        // 可视化轨迹后再清空搜索列表
                        // ROS_INFO("[\033[34mPlanNode\033[0m]: reset planning done");
                    }
                }
                if (re_plan_cnt >= REPLAN_CNT_TH){
                    ROS_INFO("[\033[31mPlanNode\033[0m]: replan failed");
                    ROS_INFO("[\033[31mPlanNode\033[0m]: giveup planner ,peace and love");
                }
            }
        }
        //// step 轨迹的优化 ##########################################################################################################
        if (!_REACH_GOAL && _OPT_TRAJ){
            if (_TRACKING){
                // step 1 Pop 待优化轨迹
                if (OptimalSegPop()) { // 待优化的轨迹不为空 进行优化
                    // auto start_time = std::chrono::steady_clock::now();
                    if (!nloptopt.NLoptSolve()) {
                        ROS_INFO("[\033[31mPlanNode\033[0m]: optimal trajectory failed");
                    } else {
                        // step 2 Push 优化后轨迹
                        ROS_INFO("[\033[32mPlanNode\033[0m]: optimal trajectory success");
                        OptimalSegPush();
                        tracking.insertSegTraj(tracking._opt_start_seg_index,&optimalTraj);
                    }
                }
            }
        }
        JMUP_REPLAM:
        // step 可视化 ############################################################################################################
        // 可视化机器人和感知范围
        if (_vis_Robot){
            visRobot(currodometry);
            // ROS_INFO("[\033[32mPlanNode\033[0m]: vis robot");
            _vis_Robot=false;
        }
        if (_vis_obs_pcl){
            visObsMap();
        }
        if (_vis_traj){
            visPtraj();
            visVtraj();
            _vis_traj=false;
        }
        if (_vis_tracking_traj){
            visTrackingTraj();
        }
        if (_vis_real_traj) {
            visRealTraj();
        }
        status = ros::ok();
        rate.sleep();
    }

    tracking.udp_bridge.closeUDP();
    return 0;
}

/************************************************************************************************************
 * @description: 期望路标点回调函数,进行路径规划,收到新的路标点后,舍弃原本目标,立马重新规划
 * @reference: 
 * @param {Path &} wp 期望路标点
 * @return {*}
 */
void rcvWaypointsCallback(const nav_msgs::Path &wp)
{
    if (_PURE_TRACKING){
        _HAS_PATH = true;
        _NEW_PATH = true;
        _REACH_GOAL = false;
        _TRACKING = true;
        Eigen::Vector3d goalPose;
        goalPose << tracking.pxtraj.back(),tracking.pytraj.back(),tracking.pqtraj.back();
        tracking._start_time = ros::Time::now().toNSec();
        tracking._goalPose = goalPose;
        ROS_INFO("[\033[32mPlanNode\033[0m]: goalPose: [%2.4f,%2.4f,%2.4f]",
            tracking._goalPose(0),tracking._goalPose(1),tracking._goalPose(2));
        if (_vis_tracking_traj) visTrackingTraj();
        return;
    }
    double yaw_rad,yaw_deg;
    Vector3d goalPose,currPose,currVel,currAcc,zeros_pt(0,0,0);
    goalPose(0) = wp.poses[0].pose.position.x;
    goalPose(1) = wp.poses[0].pose.position.y;
    goalPose(2) = tf::getYaw(wp.poses[0].pose.orientation); // use tf2 library to get yaw from quaternion
    yaw_rad = goalPose(2);
    yaw_deg = yaw_rad * 180 / M_PI;
    tracking.getCurrentState(&currPose,&currVel,&currAcc);

    // ROS_INFO("[\033[34mPlanNode\033[0m] receive the planning target");
    // ROS_INFO("[\033[34mPlanNode\033[0m]:currPose:  %2.2f, %2.2f, %2.2f", currPose(0), currPose(1), currPose(2));
    // ROS_INFO("[\033[34mPlanNode\033[0m]:currVel:   %2.2f, %2.2f, %2.2f", currVel(0), currVel(1), currVel(2));
    // ROS_INFO("[\033[34mPlanNode\033[0m]:currAcc:   %2.2f, %2.2f, %2.2f", currAcc(0), currAcc(1), currAcc(2));
    ROS_INFO("[\033[34mTarget\033[0m]:position:    %2.2f, %2.2f, %2.2f", wp.poses[0].pose.position.x, wp.poses[0].pose.position.y, wp.poses[0].pose.position.z);
    ROS_INFO("[\033[34mTarget\033[0m]:orientation: %2.2f, %2.2f, %2.2f, %2.2f", wp.poses[0].pose.orientation.x, wp.poses[0].pose.orientation.y, wp.poses[0].pose.orientation.z, wp.poses[0].pose.orientation.w);
    ROS_INFO("[\033[34mTarget\033[0m]: x:%2.2f, y:%2.2f, q:%2.2frad, q:%2.2fdeg", goalPose(0), goalPose(1), yaw_rad, yaw_deg);
    // ####################################################################################
    _HAS_PATH = false; // new goal need replan
    int re_plan_cnt = 0;
    while (_HAS_PATH == false && re_plan_cnt < SEARCH_CNT_TH && _OPT_TRAJ){
        lazykinoPRM.sample(); // 设置 Goal 后重新采用
        re_plan_cnt++;
        lazykinoPRM.reset();
        // ROS_INFO("[\033[34mTarget\033[0m]:yaw angle %f", yaw_deg);
        _HAS_PATH = !lazykinoPRM.search(currPose,currVel,currAcc,goalPose,zeros_pt,zeros_pt);
        if (_vis_sample)
            visLazyPRM();
        if (_HAS_PATH) {
            _NEW_PATH = true;
            _TRACKING = true;
            _REACH_GOAL = false;
            tracking.initPlanner(goalPose);
            tracking._goalPose = lazykinoPRM._goal_pose;
            if (!OSQPSegPush()) {
                ROS_INFO("[\033[31mPlanNode\033[0m]: search trajectory failed");
                _HAS_PATH = false;
                lazykinoPRM.sample();
                continue;
            }
            tracking.insertSegTraj(0,&searchTraj);
            // //// visulization &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
            // uint32_t num = static_cast<int>(tracking.pxtraj.size());
            // std::vector<double> time(num);
            // for (int i = 0; i < num; i++) {
            //     time[i] = i * _time_interval;
            // }
            // int dot_num = static_cast<int>(tracking.StatesSegSets.size());
            // std::vector<double> dottime(dot_num + 1);
            // std::vector<double> dotposq(dot_num + 1);
            // dottime[0] = 0;
            // dotposq[0] = tracking.StatesSegSets[0].startState.col(0)(2);
            // // std::cout   << "dotposq[0]: " << tracking.StatesSegSets[0].startState.col(0)(2)
            // //             << " " << tracking.StatesSegSets[0].startState.col(0).tail(1) << std::endl;
            // for (int i = 0; i < dot_num; i++) {
            //     dottime[i + 1] = dottime[i] + tracking.StatesSegSets[i].duration;
            //     dotposq[i + 1] = tracking.StatesSegSets[i].endState.col(0)(2);
            // }
            // plt::figure_size(1200, 800);
            // plt::plot(tracking.pxtraj, tracking.pytraj,"g.");
            // plt::title("[x.y] position figure");
            // plt::save("/home/zwt/motion_ws/src/navigation/xyposition.png"); // 保存图像
            // plt::figure_size(1200, 800);
            // // Plot line from given x and y data. Color is selected automatically.
            // plt::plot(time, tracking.pqtraj,"b-");
            // // plt::scatter(dottime, dotposq, 2.0, "rx");
            // plt::plot(dottime, dotposq,"r--");
            // // Add graph title
            // plt::title("[q] position figure");
            // plt::save("/home/zwt/motion_ws/src/navigation/qposition.png"); // 保存图像
            // plt::figure_size(1200, 800);
            // // Plot line from given x and y data. Color is selected automatically.
            // plt::plot(time, tracking.vxtraj,"r-");
            // plt::plot(time, tracking.vytraj,"g-");
            // plt::plot(time, tracking.vqtraj,"b-");
            // // Add graph title
            // plt::title("velocity figure");
            // // Enable legend.
            // // plt::legend({"vx", "vy", "vq"});
            // // plt::show(); // 显示图像(阻塞 并且显示所有图像)
            // plt::save("/home/zwt/motion_ws/src/navigation/velocity.png"); // 保存图像
            //// &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
            //// ROS_INFO("[\033[32mPlanNode\033[0m]: insertSegTraj success");
            if (OptimalSegPop()) { // 待优化的轨迹不为空 进行优化
                // auto start_time = std::chrono::steady_clock::now();
                if (!nloptopt.NLoptSolve()) {
                    ROS_INFO("[\033[31mPlanNode\033[0m]: optimal trajectory failed");
                } else {
                    // step 2 Push 优化后轨迹
                    // auto end_time = std::chrono::steady_clock::now();
                    // auto opt_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
                    // ROS_INFO("[\033[32mPlanNode\033[0m]: optimal trajectory success, time: %ld ms", opt_time.count());
                    ROS_INFO("[\033[32mPlanNode\033[0m]: optimal trajectory success");
                    OptimalSegPush();
                    tracking.insertSegTraj(tracking._opt_start_seg_index,&optimalTraj);
                }
            }
            else {
                ROS_INFO("[\033[31mPlanNode\033[0m]: optimal trajectory too short");
            }
            // save trajectory as .CSV
            // csvPtraj();
        }
        else {
            // ROS_INFO("[\033[31mPlanNode\033[0m]: search trajectory failed");
            lazykinoPRM.sample();
        }
    }

    if (re_plan_cnt >= SEARCH_CNT_TH || !_OPT_TRAJ){
        lazykinoPRM.reset();
        // ROS_INFO("[\033[34mTarget\033[0m]:yaw angle %f", yaw_deg);
        _HAS_PATH = !lazykinoPRM.search(currPose,currVel,currAcc,goalPose,zeros_pt,zeros_pt);
        if (_vis_sample)
            visLazyPRM();
        if (_HAS_PATH) {
            _NEW_PATH = true;
            _TRACKING = true;
            _REACH_GOAL = false;
            tracking.initPlanner(goalPose);
            tracking._goalPose = lazykinoPRM._goal_pose;
            if (_OPT_TRAJ) {
                if (!OSQPSegPush()) {
                    ROS_INFO("[\033[31mPlanNode\033[0m]: OSQP trajectory failed");
                    lazykinoPRM.sample();
                    SearchSegPush();
                }
            }
            else
                SearchSegPush();
            tracking.insertSegTraj(0,&searchTraj);
        }
        else {
            ROS_INFO("[\033[31mPlanNode\033[0m]: search trajectory failed");
            lazykinoPRM.sample();
        }
    }
    // ####################################################################################
    // 可视化轨迹后再清空搜索列表
    // ROS_INFO("[\033[34mPlanNode\033[0m]: reset planning done");
}

// 点云回调函数，负责设置障碍物
void rcvPointCloudCallback(const sensor_msgs::PointCloud2 & pointcloud_map)
{   
    _map_sub_cnt--;
    if (_map_sub_cnt < 0)
        _map_sub_cnt = 5;

    if (_PURE_TRACKING) return;
    pcl::PointCloud<pcl::PointXYZ> cloud;
    sensor_msgs::PointCloud2 map_vis;

    pcl::fromROSMsg(pointcloud_map, cloud);

    // ROS_DEBUG("[Node]cloud.points.size() = %d", (int)cloud.points.size());
    
    if( cloud.points.size() == 0 ) { // 如果点云为空，就不进行地图更新，但是可能是正常的(没有障碍物)
        ROS_INFO("[\033[33m !!!!! rcvPCL none points !!!!!\033[0m]:");
        return;
    }

    _HAS_MAP = true;
    // 先重置局部地图 再更新障碍物
    if (_LOCAL_PCL)
        lazykinoPRM.resetLocalMap(tracking._current_odom,_local_width);
    else 
        lazykinoPRM.raw_pcl_map->setTo(255);
    lazykinoPRM.updataObsMap(cloud);
    const cv::Mat* obsimg = lazykinoPRM.getFatMap();
    // cv::Mat obsptr = *lazykinoPRM.obs_map;
    
    // ROS_DEBUG("[Node]obsimg.size() = %d, %d", obsimg->rows, obsimg->cols);
    
    // 这里好像没必要打印obsimg,因为sdf_img已经包含了obsimg的信息
    // ROS_DEBUG("[\033[34mNode\033[0m]obsptr.size() = %d, %d", obsptr.rows, obsptr.cols);
    // ROS_DEBUG("obsimg.at(%d,%d) = %d", 100,100,obsimg.at<uchar>(100,100));
    if (_vis_obs_img) {
        sensor_msgs::ImagePtr obs_image_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", *obsimg).toImageMsg();
        _obsmap_img_pub.publish(obs_image_msg);
    }
    const cv::Mat* sdf_map_ptr = lazykinoPRM.getSdfMap();

    // ####################################################################################
    // 更新SDF地图
    nloptopt.updateEDFMap(sdf_map_ptr);
    // ####################################################################################
    if (!_vis_sdf_img) return; // 不可视化SDF地图 直接返回
    cv::Mat sdf_img = sdf_map_ptr->clone();
    cv::Mat I_norm;
    //########################################################################
    // double minVal, maxVal;
    // cv::minMaxLoc(sdf_img, &minVal, &maxVal);
    // cout << "after update obsmap" << "minVal: " << minVal << " maxVal: " << maxVal << endl;
    // 手动设置sdf地图显示的阈值 为了好看 出BUG了,直接操纵cv::Mat的指针导致sdf_map的值被改变
    // cv::threshold(*sdf_img, *sdf_img, 2, 2, THRESH_TRUNC);
    cv::normalize(sdf_img, I_norm, 0, 255, cv::NORM_MINMAX,CV_8UC1);
    cv::Mat rgb;
    cv::Mat reverse_I_norm = 255 - I_norm;
    cv::applyColorMap(reverse_I_norm, rgb, COLORMAP_COOL);
    //########################################################################
    // 计算轮廓并绘制 可有可无
    // std::vector<std::vector<cv::Point>> contours;
    // cv::findContours(I_norm, contours, cv::RETR_CCOMP, cv::CHAIN_APPROX_SIMPLE);
    // cv::drawContours(rgb, contours,-1, cv::Scalar(0, 0, 0), 1);
    cv::Mat obs_rgb;
    cv::cvtColor(*obsimg, obs_rgb, cv::COLOR_GRAY2RGB);
    cv::Mat merge_img;
    // 根据色彩值 [0=black 1=white] 进行与运算，得到obs_map与sdf_map的重叠地图
    cv::bitwise_and(rgb,obs_rgb,merge_img);
    //########################################################################
    // opencv的色彩空间是bgr，ros的色彩空间是rgb(或者相反)
    // sensor_msgs::Image is bgr8
    // cv::cvtColor(rgb, rgb, cv::COLOR_RGB2BGR);
    // attention: cv_bridge::CvImagePtr->toImageMsg() is bgr8
    cv::flip(merge_img, merge_img, 1); 
    sensor_msgs::ImagePtr sdf_image_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", merge_img).toImageMsg();
    sdf_image_msg->header.stamp = ros::Time::now();
    _sdfmap_img_pub.publish(sdf_image_msg);
    // ####################################################################################
    //// 在点云回调函数中进行轨迹检查感觉比较合理呀,可以减少里程计回调中,轨迹跟踪的计算量
    //// 还是在主循环中进行轨迹检查比较合理呀,中断回调里面最好不要执行太多的操作
}
int info_cnt = 0;
int stance_cnt = 0;
int last_curr_time_step = 0;
// 里程计回调函数，负责设置机器人位置;进行轨迹跟踪
void rcvOdomCallback(const nav_msgs::Odometry::Ptr& msg)
{
    //// 必要的处理 标志位的设置
    _odom_sub_cnt--;
    if (_odom_sub_cnt < 0)
        _odom_sub_cnt = 5;
    currodometry = msg;
    if (_OFFESET_ODOM) {
        currodometry->pose.pose.position.x = -currodometry->pose.pose.position.x;
        currodometry->pose.pose.position.y = -currodometry->pose.pose.position.y;
    }
    _vis_Robot = true;
    _HAS_ODOM  = true;
    tracking._currodometry = currodometry;
    // ####################################################################################
    // 轨迹控制 跟踪位置/速度
    Vector3d _current_odom;
    _current_odom(0) = msg->pose.pose.position.x;
    _current_odom(1)  = msg->pose.pose.position.y;
    _current_odom(2) = tf::getYaw(msg->pose.pose.orientation); // use tf2 library to get yaw from quaternion
    // 这里注意一点, local_odom 是上次里程计的位置, current_odom 是当前接收到里程计 会在下边的轨迹跟踪中更新
    // WARN: 这样处理可能造成机器人原地踏步定位的漂移 
    tracking._local_odom = tracking._current_odom;
    if (tracking._odom_cnt < tracking._odom_cnt_th){
        tracking._odom_cnt++;
        _first_pose = (_first_pose + _current_odom)/2;
        return;
    }
    if (_FIRST_ODOM) {
        tracking._goalPose = _first_pose;
        ROS_INFO("[\033[32mOdomCallback\033[0m]: first odom: %2.2f, %2.2f, %2.2f", _first_pose(0), _first_pose(1), _first_pose(2));
        _FIRST_ODOM = false;
    }
    // _current_odom(2) 的范围是 -pi ~ pi
    // ####################################################################################
    if (_HAS_PATH && !_REACH_GOAL && _TRACKING){ // 如果有路径,但是没到终点,就进行轨迹跟踪
        tracking._tracking_cnt++;
        geometry_msgs::Pose realPose;
        realPose = currodometry->pose.pose;
        double roll, pitch, yaw;
        tf::Quaternion quat;
        tf::quaternionMsgToTF(realPose.orientation, quat);
        tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
        realPose.orientation.x = roll;
        realPose.orientation.y = pitch;
        realPose.orientation.z = yaw;
        clock_t tracktimeNow = ros::Time::now().toNSec();
        double tracktimedura = (double)(tracktimeNow - tracking._start_time) / 10e6; // ms
        realPose.orientation.w = tracktimedura;
        nav_msgs::Odometry realOdom = *currodometry;
        realOdom.pose.pose = realPose;
        tracking._real_odoms.push_back(realOdom);
        // tracking._real_poses.push_back(realPose);
        if (tracking.isReachGoal(_current_odom)){
            _REACH_GOAL = true;
            ROS_INFO("[\033[32mOdomCallback\033[0m]: reach goal");
        }
        else {
            _REACH_GOAL = false;
            // 计算当前odometry在轨迹上的位置
            tracking.OdometryIndex(_current_odom);
            // ROS_INFO("[\033[32mOdomCallback\033[0m]: tracking idx: %d", tracking._curr_time_step);
            ////&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
            // 轨迹跟踪情况的可视化
            info_cnt ++;
            if (info_cnt > 10){
                info_cnt = 0;
                ROS_INFO("[\033[32mOdomCallback\033[0m]: tracking idx: %d", tracking._curr_time_step);
                if (last_curr_time_step == tracking._curr_time_step)
                    stance_cnt++;
                else
                    stance_cnt = 0;
                last_curr_time_step = tracking._curr_time_step;
                if (stance_cnt >= 5){
                    ROS_INFO("[\033[31mOdomCallback\033[0m]: tracking failed");
                    ROS_INFO("[\033[31mOdomCallback\033[0m]: giveup tracking ,peace and love");
                    _REACH_GOAL = true;
                    tracking._end_time = ros::Time::now().toNSec();
                    tracking.showTrackResult();
                    tracking._goalPose = _current_odom;
                    stance_cnt = 0;
                    last_curr_time_step = 0;
                }
            }
            // 更新轨迹跟踪的控制序列
            tracking.NavSeqUpdate();
            if (!NavSeqCheck(&tracking._nav_seq_msg)){
                ROS_INFO("[\033[33mOdomCallback\033[0m]: NavSeqCheck failed!");
                tracking.NavSeqFixed(tracking._local_odom);
            }
        }
    }
    tracking._current_odom = _current_odom;
    if(!_HAS_PATH && _TRACKING) {
        // 如果是在跟踪任务执行中,但是没有路径,可以更新当前的 pose
        if (tracking._tracking_cnt != 0)
            tracking.OdometryIndex(_current_odom);
        // 如果没有路径,就不进行轨迹跟踪
        tracking.NavSeqFixed(tracking._local_odom);
    }
    if (_REACH_GOAL){
        _TRACKING = false;
        tracking.NavSeqFixed(tracking._goalPose);
        if (tracking._tracking_cnt != 0){
            tracking._end_time = ros::Time::now().toNSec();
            tracking.showTrackResult();
        }
    }
    //DEBUG%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    // tracking.NavSeqFixed(_first_pose);
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%//
    tracking.NavSeqPublish();
}


/*******************************************************************************************
 * @description: 当前点开始检查轨迹是否无障碍
 * @reference: 
 * @return {bool} feasible true if feasible false if not
 */
inline bool TrackTrajCheck() // TODO: add BAIS_FLAG for trajectory check NMPC 跟踪效果很好感觉没啥必要
{
    int curr_ser_traj = tracking._curr_time_step;
    bool feasible = lazykinoPRM.LongTrajCheck(&tracking.pxtraj,&tracking.pytraj,&curr_ser_traj);
    if (!feasible) {
        tracking.setObsTrajPoint(curr_ser_traj);
    }
    return feasible;
}

/*******************************************************************************************
 * @description: 检查mpc跟踪轨迹是否无障碍
 * @reference: 
 * @return {bool} feasible: true if feasible | false if not
 */
bool NavSeqCheck(const nav_msgs::Path *navseq)
{
    bool feasible = true;
    double _x_start = navseq->poses[0].pose.position.x;
    double _y_start = navseq->poses[0].pose.position.y;
    for (int idx = 1; idx < static_cast<int>(navseq->poses.size()); idx++) {
        double _x_end = navseq->poses[idx].pose.position.x;
        double _y_end = navseq->poses[idx].pose.position.y;
        double dist = sqrt(pow(_x_start - _x_end, 2) + pow(_y_start - _y_end, 2));
        double _angle = atan2(_y_end - _y_start, _x_end - _x_start);
        double _cos_angle = cos(_angle);
        double _sin_angle = sin(_angle);
        vector<double> pxtraj, pytraj;
        pxtraj.push_back(_x_start);
        pytraj.push_back(_y_start);
        for (int idy = 1; idy < (dist / _DIST_RES); idy++) {
            pxtraj.push_back(_x_start + idy * _DIST_RES * _cos_angle);
            pytraj.push_back(_y_start + idy * _DIST_RES * _sin_angle);
        }
        _x_start = _x_end;  _y_start = _y_end;
        int _index = 0;
        feasible = lazykinoPRM.LongTrajCheck(&pxtraj,&pytraj,&_index);
        if (!feasible) {
            ROS_WARN("[\033[31mPlannerNode\033[0m]: NavSeqCheck failed!");
            return feasible;
        }
    }
    return feasible;
}
/*******************************************************************************************
 * @description: 搜索到的 tracksegs 从 lazykinoprm 类 push 到 searchTraj 列表
 * @reference: 
 * @return {*}
 */
bool SearchSegPush()
{
    double trajectory_length = 0, trajectory_time = 0;
    if (!searchTraj.empty())
    searchTraj.clear();
    for (int idx = 0; idx < static_cast<int>(lazykinoPRM.pathstateSets.size()); idx++)
    {
        TrackSeg _trackseg;
        _trackseg.pxtraj = lazykinoPRM.pathstateSets[idx].polytraj('p', 'x', _time_interval);
        _trackseg.pytraj = lazykinoPRM.pathstateSets[idx].polytraj('p', 'y', _time_interval);
        _trackseg.pqtraj = lazykinoPRM.pathstateSets[idx].polytraj('p', 'q', _time_interval);
        _trackseg.vxtraj = lazykinoPRM.pathstateSets[idx].polytraj('v', 'x', _time_interval);
        _trackseg.vytraj = lazykinoPRM.pathstateSets[idx].polytraj('v', 'y', _time_interval);
        _trackseg.vqtraj = lazykinoPRM.pathstateSets[idx].polytraj('v', 'q', _time_interval);
        _trackseg.startState.col(0) = lazykinoPRM.pathstateSets[idx].ParePosition; 
        _trackseg.startState.col(1) = lazykinoPRM.pathstateSets[idx].PareVelocity;
        _trackseg.startState.col(2) = lazykinoPRM.pathstateSets[idx].PareAcceleration;
        _trackseg.endState.col(0) = lazykinoPRM.pathstateSets[idx].Position;
        _trackseg.endState.col(1) = lazykinoPRM.pathstateSets[idx].Velocity;
        _trackseg.endState.col(2) = lazykinoPRM.pathstateSets[idx].Acceleration;
        Eigen::VectorXd _xcoeff = Eigen::Map<Eigen::VectorXd>(lazykinoPRM.pathstateSets[idx].xpcoeff.data(), lazykinoPRM.pathstateSets[idx].xpcoeff.size());
        Eigen::VectorXd _ycoeff = Eigen::Map<Eigen::VectorXd>(lazykinoPRM.pathstateSets[idx].ypcoeff.data(), lazykinoPRM.pathstateSets[idx].ypcoeff.size());
        Eigen::VectorXd _qcoeff = Eigen::Map<Eigen::VectorXd>(lazykinoPRM.pathstateSets[idx].qpcoeff.data(), lazykinoPRM.pathstateSets[idx].qpcoeff.size());
        // _trackseg.xcoeff = Eigen::Map<Eigen::VectorXd>(lazykinoPRM.pathstateSets[idx].xpcoeff.data(), lazykinoPRM.pathstateSets[idx].xpcoeff.size());
        // _trackseg.ycoeff = Eigen::Map<Eigen::VectorXd>(lazykinoPRM.pathstateSets[idx].ypcoeff.data(), lazykinoPRM.pathstateSets[idx].ypcoeff.size());
        // _trackseg.qcoeff = Eigen::Map<Eigen::VectorXd>(lazykinoPRM.pathstateSets[idx].qpcoeff.data(), lazykinoPRM.pathstateSets[idx].qpcoeff.size());
        _trackseg.xcoeff.resize(8);
        _trackseg.ycoeff.resize(8);
        _trackseg.qcoeff.resize(8);
        int coeff_size;
        int copy_size;
        coeff_size = _xcoeff.size();
        copy_size = min(coeff_size, 8);
        _trackseg.xcoeff.head(copy_size) = _xcoeff.head(copy_size);
        coeff_size = _ycoeff.size();
        copy_size = min(coeff_size, 8);
        _trackseg.ycoeff.head(copy_size) = _ycoeff.head(copy_size);
        coeff_size = _qcoeff.size();
        copy_size = min(coeff_size, 8);
        _trackseg.qcoeff.head(copy_size) = _qcoeff.head(copy_size);

        // std::cout << "xcoeff: " << _trackseg.xcoeff << std::endl;
        // std::cout << "ycoeff: " << _trackseg.ycoeff << std::endl;
        // std::cout << "qcoeff: " << _trackseg.qcoeff << std::endl;
        _trackseg.duration = lazykinoPRM.pathstateSets[idx].referT;

        searchTraj.push_back(_trackseg);
        trajectory_length += lazykinoPRM.pathstateSets[idx].trajectory_length;
        trajectory_time += lazykinoPRM.pathstateSets[idx].referT;
    }
    ROS_INFO("[\033[32mPlanNode\033[0m]: search trajectory length: %2.4f", trajectory_length);
    ROS_INFO("[\033[32mPlanNode\033[0m]: search trajectory time: %2.4f", trajectory_time);
    return true;
}
/*******************************************************************************************
 * @description: 搜索到的 tracksegs 使用OSQP优化后 从 lazykinoprm 类 push 到 searchTraj 列表
 * @reference: 
 * @return {bool} true if success false if not
 */
bool OSQPSegPush()
{
    bool flag = true;
    double trajectory_length = 0, trajectory_time = 0;
    if (!searchTraj.empty())
    searchTraj.clear();
    int piece_num = lazykinoPRM.pathstateSets.size() ;
    // step: 0 重置OSQPopt 设置优化段数
    OSQPopt.reset(piece_num);
    std::vector<Eigen::Vector3d> _waypoints;
    Eigen::Matrix<double, 3, 3> _startStates, _endStates;
    _startStates.col(0) = lazykinoPRM.pathstateSets.front().ParePosition;
    _startStates.col(1) = lazykinoPRM.pathstateSets.front().PareVelocity;
    _startStates.col(2) = lazykinoPRM.pathstateSets.front().PareAcceleration;
    _waypoints.push_back(lazykinoPRM.pathstateSets.front().ParePosition);
    for (int idx = 0; idx < piece_num; idx++) {
        _waypoints.push_back(lazykinoPRM.pathstateSets[idx].Position);
        trajectory_length += lazykinoPRM.pathstateSets[idx].trajectory_length;
        trajectory_time += lazykinoPRM.pathstateSets[idx].referT;
    }
    _endStates.col(0) = lazykinoPRM.pathstateSets.back().Position;
    _endStates.col(1) = lazykinoPRM.pathstateSets.back().Velocity;
    _endStates.col(2) = lazykinoPRM.pathstateSets.back().Acceleration;

    // cout.setf(ios::fixed); 
    // cout << setprecision(4) ;
    // std::cout << "LazyPRM q: " ;
    // for (int idx = 0; idx < piece_num + 1; idx++) {
    //     std::cout << _waypoints[idx](2) << " ";
    // }
    // std::cout << std::endl;
    
    OSQPopt.setWaypoints(_waypoints,_startStates, _endStates);

    // std::cout << "OSQPopt waypoints: " ;
    // std::cout << OSQPopt.Waypoints.row(2) << std::endl;

    // for (int idx = 0; idx < piece_num + 1; idx++) {
    //     std::cout << OSQPopt.Waypoints.row(2) << " ";
    // }
    // std::cout << std::endl;

    // std::cout << "OSQPopt segments: " << OSQPopt.N << std::endl;
    // std::cout << "OSQPopt startState :" << std::endl;
    // std::cout << OSQPopt.StartStates << std::endl;
    // std::cout << "OSQPopt endState :" << std::endl;
    // std::cout << OSQPopt.EndStates << std::endl;
    // cout.unsetf(ios::fixed);
    // step: 1 OSQP 优化求解 // OSQPSolve [x,y,q] | OSQPSolveDiv [x],[y],[q] 
    if(!OSQPopt.OSQPSolveDiv()) {
        return false;
    }
    // step: 2 优化后的轨迹段 push 到 searchTraj 列表
    for (int idx = 0; idx < piece_num; idx++) {
        TrackSeg _trackseg;
        double _duration = OSQPopt.Traj[idx].getDuration();
        _trackseg.duration = _duration;
        _trackseg.startState.col(0) = OSQPopt.Traj[idx].getPos(0.0); 
        _trackseg.startState.col(1) = OSQPopt.Traj[idx].getVel(0.0);
        _trackseg.startState.col(2) = OSQPopt.Traj[idx].getAcc(0.0);
        _trackseg.endState.col(0) = OSQPopt.Traj[idx].getPos(_duration);
        _trackseg.endState.col(1) = OSQPopt.Traj[idx].getVel(_duration);
        _trackseg.endState.col(2) = OSQPopt.Traj[idx].getAcc(_duration);
        // &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
        _trackseg.xcoeff = OSQPopt.Traj[idx].getCoeffMat().row(0).transpose();
        _trackseg.ycoeff = OSQPopt.Traj[idx].getCoeffMat().row(1).transpose();
        _trackseg.qcoeff = OSQPopt.Traj[idx].getCoeffMat().row(2).transpose();
        // &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
        int num = static_cast<int>(_duration / _time_interval);
        for (int idy = 0; idy < num; idy++) {
            double t = idy * _time_interval;
            _trackseg.pxtraj.push_back(OSQPopt.Traj[idx].getPos(t)(0));
            _trackseg.pytraj.push_back(OSQPopt.Traj[idx].getPos(t)(1));
            _trackseg.pqtraj.push_back(OSQPopt.Traj[idx].getPos(t)(2));
            _trackseg.vxtraj.push_back(OSQPopt.Traj[idx].getVel(t)(0));
            _trackseg.vytraj.push_back(OSQPopt.Traj[idx].getVel(t)(1));
            _trackseg.vqtraj.push_back(OSQPopt.Traj[idx].getVel(t)(2));
        }
        int _index = 0;
        if(!lazykinoPRM.LongTrajCheck(&_trackseg.pxtraj,&_trackseg.pytraj,&_index)) {
           flag = false;
        }
        searchTraj.push_back(_trackseg);
    }
    // std::cout << "OSQPopt time: " << OSQPopt.initT.transpose() << std::endl;
    ROS_INFO("[\033[32mPlanNode\033[0m]: OSQPopt trajectory length: %2.4f", trajectory_length);
    ROS_INFO("[\033[32mPlanNode\033[0m]: OSQPopt trajectory time: %2.4f", trajectory_time);

    if (_vis_osqp_traj) 
        visOSQPTraj();
    return flag;
}

/*******************************************************************************************
 * @description: 待优化的 tracksegs 从 Tracking tracking 类 pop 到 optimalTraj 列表 并设置优化初始值
 * @reference: 
 * @return {*}
 */
bool OptimalSegPop()
{
    if (!optimalTraj.empty())
    optimalTraj.clear();
    if (!tracking.popOptSegTraj(tracking._opt_start_seg_index, &optimalTraj))
        return false;
    int piece_num = optimalTraj.size();
    // // step: 0 重置 nloptopt 设置优化段数
    // if (piece_num == 0) { // 如果没有轨迹段,就不进行优化
    //     return false;
    // }
    nloptopt.reset(piece_num);
    _opt_init.resize(piece_num * 25); // dimensions * order + time = 3 * 8 + 1 = 25
    std::vector<Eigen::Vector3d> _waypoints;
    std::vector<double> _initt;
    Eigen::Matrix<double, 3, 3> _startStates, _endStates;
    _startStates = optimalTraj.front().startState;
    _waypoints.push_back(optimalTraj[0].startState.col(0));
    for (int idx = 0; idx < piece_num; idx++) {
        _waypoints.push_back(optimalTraj[idx].endState.col(0));
        _initt.push_back(optimalTraj[idx].duration);
        _opt_init.segment(idx * 24,24) <<   optimalTraj[idx].xcoeff, 
                                            optimalTraj[idx].ycoeff, 
                                            optimalTraj[idx].qcoeff;
        // std::cout << "xcoeff: " << optimalTraj[idx].xcoeff.transpose() << std::endl;
        // std::cout << "ycoeff: " << optimalTraj[idx].ycoeff.transpose() << std::endl;
        // std::cout << "qcoeff: " << optimalTraj[idx].qcoeff.transpose() << std::endl;
    }
    _endStates = optimalTraj.back().endState;
    //// Acc set as zero
    _startStates.col(2) = _startStates.col(2) * 0.0;
    _endStates.col(2) = _endStates.col(2) * 0.0;
    //// nlopt local optimization set waypoints without any modification
    if(!nloptopt.pushWaypoints(_waypoints,_initt,_startStates, _endStates)){
        ROS_WARN("[\033[31mPlannerNode\033[0m]: pushWaypoints failed!");
        return false;
    }
    // step: 1 设置优化初始值
    // std::cout << "_opt_init full: " << _opt_init.transpose() << std::endl;
    nloptopt.getReduceOrder(_opt_init);
    // std::cout << "_opt_init reduce : " << _opt_init.transpose() << std::endl;
    Eigen::VectorXd _inittau = Eigen::Map<const Eigen::VectorXd>(_initt.data(), _initt.size());
    //// any difference between _inittau.exp() and _inittau.array().exp() ?
    // std::cout << "_inittau: " << _inittau.transpose() << std::endl;
    // std::cout << "_initlogtau: " << _inittau.array().log().transpose() << std::endl;
    _opt_init.tail(piece_num) = _inittau.array().log();
    nloptopt.updateOptVars(_opt_init);
    return true;
}

/*******************************************************************************************
 * @description: 优化后的 tracksegs 从 NonTrajOpt nloptopt 类 push 到 optimalTraj 列表中
 * @reference: 
 * @return {*}
 */
void OptimalSegPush()
{
    double trajectory_time = 0;
    if (!optimalTraj.empty())
    optimalTraj.clear();
    int piece_num = nloptopt.Traj.getPieceNum();
    // step: 2 优化后的轨迹段 push 到 optimalTraj 列表
    for (int idx = 0; idx < piece_num; idx++) {
        TrackSeg _trackseg;
        double _duration = nloptopt.Traj[idx].getDuration();
        _trackseg.duration = _duration;
        _trackseg.startState.col(0) = nloptopt.Traj[idx].getPos(0.0); 
        _trackseg.startState.col(1) = nloptopt.Traj[idx].getVel(0.0);
        _trackseg.startState.col(2) = nloptopt.Traj[idx].getAcc(0.0);
        _trackseg.endState.col(0) = nloptopt.Traj[idx].getPos(_duration);
        _trackseg.endState.col(1) = nloptopt.Traj[idx].getVel(_duration);
        _trackseg.endState.col(2) = nloptopt.Traj[idx].getAcc(_duration);
        // &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
        _trackseg.xcoeff = nloptopt.Traj[idx].getCoeffMat().row(0).transpose();
        _trackseg.ycoeff = nloptopt.Traj[idx].getCoeffMat().row(1).transpose();
        _trackseg.qcoeff = nloptopt.Traj[idx].getCoeffMat().row(2).transpose();
        // &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
        int num = static_cast<int>(_duration / _time_interval);
        for (int idy = 0; idy < num; idy++) {
            double t = idy * _time_interval;
            _trackseg.pxtraj.push_back(nloptopt.Traj[idx].getPos(t)(0));
            _trackseg.pytraj.push_back(nloptopt.Traj[idx].getPos(t)(1));
            _trackseg.pqtraj.push_back(nloptopt.Traj[idx].getPos(t)(2));
            _trackseg.vxtraj.push_back(nloptopt.Traj[idx].getVel(t)(0));
            _trackseg.vytraj.push_back(nloptopt.Traj[idx].getVel(t)(1));
            _trackseg.vqtraj.push_back(nloptopt.Traj[idx].getVel(t)(2));
        }
        optimalTraj.push_back(_trackseg);
        trajectory_time += _duration;
    }
    std::cout << "nloptopt time: " << nloptopt.Vect.transpose() << std::endl;
    ROS_INFO("[\033[32mPlanNode\033[0m]: nloptopt trajectory time: %2.4f", trajectory_time);

    if (_vis_nlopt_traj) 
        visNLOptTraj();
}

//########################################################################################
// 可视化函数

void visPtraj()
{
    return;
    nav_msgs::Path path;
    path.header.frame_id = "planner";
    path.header.stamp = ros::Time::now();
}

void visVtraj()
{
    return;
}


int csv_record_num = 0;
void csvPtraj()
{
    ofstream outfile;
    std::ostringstream filename;
    filename << "/home/zwt/catkin_ws/src/navigation/fast_navigation/datas/traj_" << csv_record_num << ".csv";
    outfile.open(filename.str());
    if (outfile)
    {
        outfile.close();
        outfile.open(filename.str(), ios_base::out | ios_base::trunc);
    }
    outfile << "px,py,pq,vx,vy,vq" << endl;
    ROS_INFO("[\033[35mNodeCSV\033[0m] datas.size() = %ld", tracking.pxtraj.size());
    for (int i = 0; i < static_cast<int>(tracking.pqtraj.size()); i++)
    {
        outfile << tracking.pxtraj.at(i) << "," << tracking.pytraj.at(i) << "," << tracking.pqtraj.at(i) << ","
             << tracking.vxtraj.at(i) << "," << tracking.vytraj.at(i) << "," << tracking.vqtraj.at(i) << endl;
    }
    outfile.close();
    csv_record_num++;
}

/*******************************************************************************************
 * @description:  可视化 Tracking 的轨迹
 * @reference: 
 * @return {*}
 */
void visTrackingTraj()
{
    if (tracking.pxtraj.empty())
        return;
    double _vis_resolution = 0.02;
    visualization_msgs::MarkerArray  LineArray;
    visualization_msgs::Marker       Line;
    Line.header.frame_id = "planner";
    Line.header.stamp    = ros::Time::now();
    Line.ns              = "planner_node/TraLibrary";
    Line.action          = visualization_msgs::Marker::ADD;

    Line.pose.orientation.w = 1.0;
    Line.type            = visualization_msgs::Marker::LINE_STRIP;
    Line.scale.x         = _vis_resolution;

    Line.color.r         = 0.0;
    Line.color.g         = 1.0;
    Line.color.b         = 0.0;
    Line.color.a         = 0.5;
    Line.id = 33;

    geometry_msgs::Point p;
    p.z = DEFAULT_HIGH;
    for (int idx = 0; idx < static_cast<int>(tracking.pxtraj.size()); idx+=5)
    {
        if (idx >= static_cast<int>(tracking.pxtraj.size()))
            break;
        p.x = tracking.pxtraj[idx];
        p.y = tracking.pytraj[idx];
        Line.points.push_back(p);
    }
    LineArray.markers.push_back(Line);    
    // ROS_INFO("[\033[32mPlanNode\033[0m]: visTrackingTraj.size() = %ld", Line.points.size());
    _track_vis_pub.publish(LineArray);
}

/*******************************************************************************************
 * @description:  可视化 Tracking 的轨迹
 * @reference: 
 * @return {*}
 */
void visRealTraj()
{
    if (tracking._real_odoms.empty())
        return;
    double _vis_resolution = 0.02;
    visualization_msgs::MarkerArray  LineArray;
    visualization_msgs::Marker       Line;
    Line.header.frame_id = "planner";
    Line.header.stamp    = ros::Time::now();
    Line.ns              = "planner_node/TraLibrary";
    Line.action          = visualization_msgs::Marker::ADD;

    Line.pose.orientation.w = 1.0;
    Line.type            = visualization_msgs::Marker::LINE_STRIP;
    Line.scale.x         = _vis_resolution;

    Line.color.r         = 1.0;
    Line.color.g         = 0.0;
    Line.color.b         = 0.0;
    Line.color.a         = 0.5;
    Line.id = 33;

    geometry_msgs::Point p;
    p.z = DEFAULT_HIGH;
    for (int idx = 0; idx < static_cast<int>(tracking._real_odoms.size()); idx++)
    {
        
        p.x = tracking._real_odoms[idx].pose.pose.position.x;
        p.y = tracking._real_odoms[idx].pose.pose.position.y;
        Line.points.push_back(p);
    }
    LineArray.markers.push_back(Line);    
    // ROS_INFO("[\033[32mPlanNode\033[0m]: visTrackingTraj.size() = %ld", Line.points.size());
    _real_vis_pub.publish(LineArray);
}

/*******************************************************************************************
 * @description:  可视化搜索到的: 轨迹 采样点 终点
 * @reference: 
 * @return {*}
 */
void visLazyPRM()
{
    double _vis_resolution = 0.04;
    visualization_msgs::MarkerArray  LineArray;
    visualization_msgs::Marker       Line;
    visualization_msgs::Marker       Spheres;
    visualization_msgs::Marker       ESpheres;
    visualization_msgs::Marker       OSpheres;
    visualization_msgs::Marker       GSphere;
    visualization_msgs::Marker       SSpheres;
    visualization_msgs::Marker       ISpheres;

    // Path State Sets Visualization
    Spheres.header.frame_id =  Line.header.frame_id = "planner";
    Spheres.header.stamp    =  Line.header.stamp    = ros::Time::now();
    Spheres.ns              =  Line.ns              = "planner_node/TraLibrary";
    Spheres.action          =  Line.action          = visualization_msgs::Marker::ADD;
    // Search Tree Visualization
    ESpheres.header.frame_id =  OSpheres.header.frame_id = "planner";
    ESpheres.header.stamp    =  OSpheres.header.stamp    = ros::Time::now();
    ESpheres.ns              =  OSpheres.ns              = "planner_node/TraLibrary";
    ESpheres.action          =  OSpheres.action          = visualization_msgs::Marker::ADD;
    // Goal Point Visualization
    GSphere.header.frame_id = "planner";
    GSphere.header.stamp    = ros::Time::now();
    GSphere.ns              = "planner_node/TraLibrary";
    GSphere.action          = visualization_msgs::Marker::ADD;
    // Sample Point Visualization Mid point
    SSpheres.header.frame_id = "planner";
    SSpheres.header.stamp    = ros::Time::now();
    SSpheres.ns              = "planner_node/TraLibrary";
    SSpheres.action          = visualization_msgs::Marker::ADD;
    // Sample Point Visualization Invalid point
    ISpheres.header.frame_id = "planner";
    ISpheres.header.stamp    = ros::Time::now();
    ISpheres.ns              = "planner_node/TraLibrary";
    ISpheres.action          = visualization_msgs::Marker::ADD;

    Line.pose.orientation.w = 1.0;
    Line.type            = visualization_msgs::Marker::LINE_STRIP;
    Line.scale.x         = _vis_resolution;

    Line.color.r         = 0.0;
    Line.color.g         = 0.0;
    Line.color.b         = 1.0;
    Line.color.a         = 0.5;

    Spheres.pose.orientation.w = 1.0;
    Spheres.type            = visualization_msgs::Marker::SPHERE_LIST;
    Spheres.scale.x         = _vis_resolution*5;
    Spheres.scale.y         = _vis_resolution*5;
    Spheres.scale.z         = _vis_resolution*5;

    Spheres.color.a         = 0.2;
    Spheres.color.r         = 0.0;
    Spheres.color.g         = 0.0;
    Spheres.color.b         = 1.0;

    ESpheres.pose.orientation.w = 1.0;
    ESpheres.type            = visualization_msgs::Marker::SPHERE_LIST;
    ESpheres.scale.x         = _vis_resolution*3;
    ESpheres.scale.y         = _vis_resolution*3;
    ESpheres.scale.z         = _vis_resolution*3;

    ESpheres.color.a         = 0.4;
    ESpheres.color.r         = 1.0;
    ESpheres.color.g         = 0.0;
    ESpheres.color.b         = 0.0;

    OSpheres.pose.orientation.w = 1.0;
    OSpheres.type            = visualization_msgs::Marker::SPHERE_LIST;
    OSpheres.scale.x         = _vis_resolution*3;
    OSpheres.scale.y         = _vis_resolution*3;
    OSpheres.scale.z         = _vis_resolution*3;

    OSpheres.color.a         = 0.4;
    OSpheres.color.r         = 0.0;
    OSpheres.color.g         = 1.0;
    OSpheres.color.b         = 0.0;
    
    GSphere.pose.orientation.w = 1.0;
    GSphere.type            = visualization_msgs::Marker::CUBE;
    GSphere.scale.x         = _vis_resolution*3;
    GSphere.scale.y         = _vis_resolution*3;
    GSphere.scale.z         = _vis_resolution*3;

    GSphere.color.a         = 1.0;
    GSphere.color.r         = 0.0;
    GSphere.color.g         = 0.0;
    GSphere.color.b         = 0.0;

    SSpheres.pose.orientation.w = 1.0;
    SSpheres.type            = visualization_msgs::Marker::SPHERE_LIST;
    SSpheres.scale.x         = _vis_resolution;
    SSpheres.scale.y         = _vis_resolution;
    SSpheres.scale.z         = _vis_resolution;

    SSpheres.color.a         = 0.8;
    SSpheres.color.r         = 1.0;
    SSpheres.color.g         = 1.0;
    SSpheres.color.b         = 1.0;

    ISpheres.pose.orientation.w = 1.0;
    ISpheres.type            = visualization_msgs::Marker::SPHERE_LIST;
    ISpheres.scale.x         = _vis_resolution;
    ISpheres.scale.y         = _vis_resolution;
    ISpheres.scale.z         = _vis_resolution;

    ISpheres.color.a         = 1.0;
    ISpheres.color.r         = 0.0;
    ISpheres.color.g         = 0.0;
    ISpheres.color.b         = 0.0;
    
    // #######################################################################################
    // Goal Point Visualization
    int marker_id = 0;
    geometry_msgs::Point pt;
    GSphere.pose.position.x = lazykinoPRM._goal_pose(0);
    GSphere.pose.position.y = lazykinoPRM._goal_pose(1);
    GSphere.pose.position.z = DEFAULT_HIGH;
    GSphere.id = marker_id;
    LineArray.markers.push_back(GSphere);
    marker_id++;
    
    // #######################################################################################
    // Path State Sets Visualization
    if (static_cast<int>(lazykinoPRM.pathstateSets.size()) == 0)
    {
        ROS_INFO("[\033[34mvisTraLibrary\033[0m]No path found!");
        // return;
    }
    else
    {
        for (int idx = 0; idx < static_cast<int>(lazykinoPRM.pathstateSets.size()); idx++)
        {
            vector<double> xtraj, ytraj;
            xtraj = lazykinoPRM.pathstateSets[idx].polytraj('p', 'x', _time_interval);
            ytraj = lazykinoPRM.pathstateSets[idx].polytraj('p', 'y', _time_interval);
            for (int i = 0; i < static_cast<int>(xtraj.size()); i = i + 5)
            {
                if (i >= static_cast<int>(xtraj.size()))
                    break;
                geometry_msgs::Point p;
                p.x = xtraj[i];
                p.y = ytraj[i];
                p.z = DEFAULT_HIGH;
                Line.points.push_back(p);
                Line.id = marker_id;
            }
        }
        LineArray.markers.push_back(Line);
        ++marker_id;

        Spheres.id = marker_id;

        for (int idx = 0; idx < static_cast<int>(lazykinoPRM.pathstateSets.size()); idx++)
        {
            Vector3d coord = lazykinoPRM.pathstateSets[idx].Position;
            pt.x = coord(0);
            pt.y = coord(1);
            pt.z = DEFAULT_HIGH;
            // ROS_INFO("[\033[34mvisTraLibrary\033[0m]Position = [%f, %f, %f]", coord(0), coord(1), coord(2));
            Spheres.points.push_back(pt);
        }
        LineArray.markers.push_back(Spheres);
        ++marker_id;
    }
    // #######################################################################################
    // Search Tree Visualization
    if (static_cast<int>(lazykinoPRM.astaropenlist.nodestateSets.size()) == 0)
    {
        ROS_INFO("[\033[34mvisTraLibrary\033[0m]No OpenList found!");
    }
    else // node of openlist
    {
        OSpheres.id = marker_id;
        ++marker_id;
        ESpheres.id = marker_id;
        ++marker_id;
        for (int64_t idx = 0; idx < static_cast<int64_t>(lazykinoPRM.astaropenlist.nodestateSets.size()); idx++)
        {
            GridNodePtr gridnodeptr = lazykinoPRM.astaropenlist.nodestateSets[idx].CurrGridNodePtr;
            Vector3d coord = lazykinoPRM.astaropenlist.nodestateSets[idx].Position;
            pt.x = coord(0);
            pt.y = coord(1);
            pt.z = DEFAULT_HIGH;
            if (gridnodeptr->type == Extend)
            {
                ESpheres.points.push_back(pt);
            }
            else if (gridnodeptr->type == Mid)
            {
                OSpheres.points.push_back(pt);
            }
        }
        LineArray.markers.push_back(OSpheres);
        LineArray.markers.push_back(ESpheres);
        ROS_DEBUG("[visLazyPRM]ESpheres.size() = %ld", OSpheres.points.size());
        ROS_DEBUG("[visLazyPRM]OSpheres.size() = %ld", ESpheres.points.size());
        // ROS_DEBUG("[visLazyPRM]marker_id = %d", marker_id);
    }
    // #######################################################################################
    // Sample Point Visualization
    SSpheres.id = marker_id;
    ++marker_id;
    ISpheres.id = marker_id;
    ++marker_id;
    ROS_DEBUG("[visLazyPRM]getSampleNum() = %d", lazykinoPRM.getSampleNum());
    for (int idx = 0; idx < lazykinoPRM.getSampleNum(); idx++)
    {
        GridNodePtr gridnodeptr = lazykinoPRM.getNodePtr(idx);
        if (gridnodeptr == nullptr)
            continue;
        Vector3d coord = gridnodeptr->pose;
        pt.x = coord(0);
        pt.y = coord(1);
        pt.z = DEFAULT_HIGH;
        if (gridnodeptr->type == Invalid)
        {
            ISpheres.points.push_back(pt);
        }
        else //if (gridnodeptr->type == Invalid)
        {
            SSpheres.points.push_back(pt);
        }
    }
    LineArray.markers.push_back(SSpheres);
    LineArray.markers.push_back(ISpheres);
    ROS_DEBUG("[visLazyPRM]SSpheres.size() = %ld", SSpheres.points.size());
    ROS_DEBUG("[visLazyPRM]ISpheres.size() = %ld", ISpheres.points.size());
    _path_vis_pub.publish(LineArray);
}

/*******************************************************************************************
 * @description:  可视化OSQP初步优化的轨迹 
 * @reference: 
 * @return {*}
 */
void visOSQPTraj()
{
    double _vis_resolution = 0.02;
    visualization_msgs::MarkerArray  LineArray;
    visualization_msgs::Marker       Line;
    visualization_msgs::Marker       Spheres;
     // Path State Sets Visualization
    Spheres.header.frame_id =  Line.header.frame_id = "planner";
    Spheres.header.stamp    =  Line.header.stamp    = ros::Time::now();
    Spheres.ns              =  Line.ns              = "planner_node/osqpVis";
    Spheres.action          =  Line.action          = visualization_msgs::Marker::ADD;

    Line.pose.orientation.w = 1.0;
    Line.type            = visualization_msgs::Marker::LINE_STRIP;
    Line.scale.x         = _vis_resolution;

    Line.color.r         = 0.0;
    Line.color.g         = 1.0;
    Line.color.b         = 0.0;
    Line.color.a         = 0.5;

    Spheres.pose.orientation.w = 1.0;
    Spheres.type            = visualization_msgs::Marker::SPHERE_LIST;
    Spheres.scale.x         = _vis_resolution*2;
    Spheres.scale.y         = _vis_resolution*2;
    Spheres.scale.z         = _vis_resolution*2;

    Spheres.color.r         = 0.0;
    Spheres.color.g         = 0.0;
    Spheres.color.b         = 1.0;
    Spheres.color.a         = 0.5;

    geometry_msgs::Point pt;
    Spheres.id = 1;
    pt.x = searchTraj.front().startState(0,0);
    pt.y = searchTraj.front().startState(1,0);
    pt.z = DEFAULT_HIGH;
    Spheres.points.push_back(pt);
    for (int idx = 0; idx < static_cast<int>(searchTraj.size()); idx++)
    {
        vector<double> xtraj, ytraj;
        xtraj = searchTraj.at(idx).pxtraj;
        ytraj = searchTraj.at(idx).pytraj;
        pt.x = searchTraj.at(idx).endState(0,0);
        pt.y = searchTraj.at(idx).endState(1,0);
        Spheres.points.push_back(pt);
        geometry_msgs::Point p;
        p.z = DEFAULT_HIGH;
        Line.id = 2;
        for (int i = 0; i < static_cast<int>(xtraj.size()); i = i + 5)
        {
            if (i >= static_cast<int>(xtraj.size()))
                break;
            
            p.x = xtraj[i];
            p.y = ytraj[i];
            Line.points.push_back(p);
        }
    }
    LineArray.markers.push_back(Line);    
    LineArray.markers.push_back(Spheres);

    _osqp_vis_pub.publish(LineArray);
}

/*******************************************************************************************
 * @description:  可视化NLopt优化的轨迹
 * @reference: 
 * @return {*}
 */
void visNLOptTraj()
{
    double _vis_resolution = 0.02;
    visualization_msgs::MarkerArray  LineArray;
    visualization_msgs::Marker       Line;
    visualization_msgs::Marker       Spheres;
     // Path State Sets Visualization
    Line.header.frame_id = "planner";
    Line.header.stamp    = ros::Time::now();
    Line.ns              = "planner_node/nloptVis";
    Line.action          = visualization_msgs::Marker::ADD;

    Line.pose.orientation.w = 1.0;
    Line.type            = visualization_msgs::Marker::LINE_STRIP;
    Line.scale.x         = _vis_resolution;

    Line.color.r         = 1.0;
    Line.color.g         = 0.0;
    Line.color.b         = 0.0;
    Line.color.a         = 0.5;

    for (int idx = 0; idx < static_cast<int>(optimalTraj.size()); idx++)
    {
        vector<double> xtraj, ytraj;
        xtraj = optimalTraj.at(idx).pxtraj;
        ytraj = optimalTraj.at(idx).pytraj;
        geometry_msgs::Point p;
        p.z = DEFAULT_HIGH;
        Line.id = 3;
        for (int i = 0; i < static_cast<int>(xtraj.size()); i = i + 5)
        {
            if (i >= static_cast<int>(xtraj.size()))
                break;
            
            p.x = xtraj[i];
            p.y = ytraj[i];
            Line.points.push_back(p);
        }
    }
    LineArray.markers.push_back(Line);    

    _nlopt_vis_pub.publish(LineArray);
}

/*******************************************************************************************
 * @description: 可视化lazykinoPRM中的障碍物地图
 * @reference: 
 * @return {*}
 */
void visObsMap()
{
    pcl::PointXYZ pt_image;
    const cv::Mat* obsimg = lazykinoPRM.getFatMap();
    if (obsimg == nullptr)
        return;
    sensor_msgs::PointCloud2 _obs_map_pcl_msg; // 全局障碍物点云
    pcl::PointCloud<pcl::PointXYZ> _obs_map_pcl; // 全局障碍物点云
    uint16_t img_x,img_y,idx,idy;
    double _x_orign, _y_orign;
    double _resolution = lazykinoPRM.xy_resolution_;
    img_x = obsimg->cols;
    img_y = obsimg->rows;
   _x_orign = img_x*_resolution/2;
   _y_orign = img_y*_resolution/2;
   _obs_map_pcl.points.clear();
   for (idx = 0; idx < img_x; idx = idx + 1) {
      for (idy = 0; idy < img_y; idy = idy + 1) {
         // 如果该点有像素值 就是point, 使用Point()函数访问图像的像素值
         if (obsimg->at<uchar>(Point(idx,idy)) == 0){
            pt_image.x = idx * _resolution - _x_orign;
            pt_image.y = idy * _resolution - _y_orign;
            pt_image.z = DEFAULT_HIGH;
            _obs_map_pcl.points.push_back(pt_image);
         }
      }
   }
    _obs_map_pcl.width = _obs_map_pcl.points.size();
    _obs_map_pcl.height = 1;
    _obs_map_pcl.is_dense = true;
    pcl::toROSMsg(_obs_map_pcl, _obs_map_pcl_msg);
    _obs_map_pcl_msg.header.stamp = ros::Time::now();
    _obs_map_pcl_msg.header.frame_id = "planner";
    // ROS_INFO("[\033[34mvisObsMap\033[0m]obs_map_pcl_msg.size() = %ld", _obs_map_pcl_msg.data.size());
    _obs_map_pub.publish(_obs_map_pcl_msg);
}

/*******************************************************************************************
 * @description:  可视化 robot : body | local window | persuit point
 * @reference: 
 * @param {ConstPtr&} msg
 * @return {*}
 */
void visRobot(const nav_msgs::Odometry::Ptr& msg)
{   
    // 获取机器人当前位姿
    if (msg == nullptr)
        return;
    geometry_msgs::Pose pose = msg->pose.pose;
    double _vis_resolution = 0.02;
    visualization_msgs::MarkerArray  LineArray;
    visualization_msgs::Marker       Line;
    visualization_msgs::Marker       Body;
    visualization_msgs::Marker       Spheres;

    // if(_OFFESET_ODOM) {
    //     pose.position.x = -pose.position.x;
    //     pose.position.y = -pose.position.y;
    // }
    
    Body.header.frame_id =  Line.header.frame_id = "planner";
    Body.header.stamp    =  Line.header.stamp    = ros::Time::now();
    Body.ns              =  Line.ns              = "planner_node/visRobot";
    Body.action          =  Line.action          = visualization_msgs::Marker::ADD;

    Spheres.header.frame_id = "planner";
    Spheres.header.stamp    = ros::Time::now();
    Spheres.ns              = "planner_node/visRobot";
    Spheres.action          = visualization_msgs::Marker::ADD;

    Line.id              = 1;
    Line.pose.orientation.w = 1.0;
    Line.type            = visualization_msgs::Marker::LINE_STRIP;
    Line.scale.x         = _vis_resolution;

    Line.color.r         = 0.0;
    Line.color.g         = 0.0;
    Line.color.b         = 1.0;
    Line.color.a         = 1.0;

    Body.id              = 2;
    Body.pose.orientation.w = 1.0;
    Body.type            = visualization_msgs::Marker::SPHERE;
    Body.scale.x         = DEFAULT_LENGTH;
    Body.scale.y         = DEFAULT_WIDTH;
    Body.scale.z         = DEFAULT_HIGH/2;

    Body.color.a         = RGB_ALPHA;
    Body.color.r         = 0.0;
    Body.color.g         = 0.0;
    Body.color.b         = 1.0;

    Spheres.id              = 3;
    Spheres.pose.orientation.w = 1.0;
    Spheres.type            = visualization_msgs::Marker::SPHERE;
    Spheres.scale.x         = _vis_resolution*5;
    Spheres.scale.y         = _vis_resolution*5;
    Spheres.scale.z         = _vis_resolution*5;

    Spheres.color.r         = 0.0;
    Spheres.color.g         = 1.0;
    Spheres.color.b         = 0.0;
    Spheres.color.a         = 1.0;

    // 可视化机器人的感知范围
    geometry_msgs::Point pt;
    pt.x = pose.position.x - _local_width;
    pt.y = pose.position.y - _local_width;
    pt.z = DEFAULT_HIGH;
    Line.points.push_back(pt);
    pt.x = pose.position.x + _local_width;
    pt.y = pose.position.y - _local_width;
    pt.z = DEFAULT_HIGH;
    Line.points.push_back(pt);
    pt.x = pose.position.x + _local_width;
    pt.y = pose.position.y + _local_width;
    pt.z = DEFAULT_HIGH;
    Line.points.push_back(pt);
    pt.x = pose.position.x - _local_width;
    pt.y = pose.position.y + _local_width;
    pt.z = DEFAULT_HIGH;
    Line.points.push_back(pt);
    pt.x = pose.position.x - _local_width;
    pt.y = pose.position.y - _local_width;
    pt.z = DEFAULT_HIGH;
    Line.points.push_back(pt);
    LineArray.markers.push_back(Line);

    // 可视化机器人的当前位姿
    Body.pose.position.x = pose.position.x;
    Body.pose.position.y = pose.position.y;
    Body.pose.position.z = pose.position.z;
    Body.pose.orientation.x = pose.orientation.x;
    Body.pose.orientation.y = pose.orientation.y;
    Body.pose.orientation.z = pose.orientation.z;
    Body.pose.orientation.w = pose.orientation.w;

    LineArray.markers.push_back(Body);

    //可视化下一个目标点
    Spheres.pose.position.x = tracking._pursuitPose(0);
    Spheres.pose.position.y = tracking._pursuitPose(1);
    Spheres.pose.position.z = DEFAULT_HIGH;
    Spheres.pose.orientation.w = 1.0;

    LineArray.markers.push_back(Spheres);

    _robot_vis_pub.publish(LineArray);
}

/********************************************************************************/