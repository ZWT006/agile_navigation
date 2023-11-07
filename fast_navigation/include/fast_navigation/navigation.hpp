/*
 * @Author: wentao zhang && zwt190315@163.com
 * @Date: 2023-06-13
 * @LastEditTime: 2023-11-07
 * @Description: 
 * @reference: 
 * 
 * Copyright (c) 2023 by wentao zhang, All Rights Reserved. 
 */
#ifndef _NAVIGATION_ASTAR_H
#define _NAVIGATION_ASTAR_H

#include <time.h>
#include <iostream>
#include <fstream>
#include <math.h>
#include <angles/angles.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>

#include <tf/transform_datatypes.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

#include <Eigen/Eigen>
#include <vector>

#include <fast_navigation/UDPbridge.hpp>

// lazykinoprm path searching
// #include <lazykinoprm/LazyKinoPRM.h>
// nontrajopt trajectory optimization
// #include <nontrajopt/nontrajopt.h>

// nearPose judeg threshold
#define DIST_XY 0.03f
#define DIST_Q 0.0872646f

// 速度跟踪的实际缩放因子 机器人实际的状态估计速度值好像比较小
#define VEL_SCALE 0.0f
#define ANG_SCALE 0.0f

// robot visulaization param
//unitree A1 robot size
#define STAND_HIGH 0.30f

// max local bias step 这个限度内就正常推进 persuit 前移 
// 感觉这个参数可以设置的大一点,因为 NMPC 滚动优化的时候会有偏差修正
#define AMX_BIAS_STEP 3

#define PURSET_STEP_BACK_BIAS 3 // persuit 在轨迹上的定位后退步长

#define REPLAN_BIAS 2 // 重规划偏差阈值 当前seg_index + REPLAN_BIAS 为重规划起始段
#define NLOPT_BIAS 2    // 优化偏差阈值 当前seg_index + OPT_BIAS 为优化起始段

//// &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
#define LOCAL_PORT 1024
#define LOCAL_IP_ADDRESS    "127.0.0.1" //"192.168.2.240"
#define DEST_PORT 2048
#define DSET_IP_ADDRESS     "127.0.0.1"
//// &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&

bool nearPose(Eigen::Vector2d currPose,Eigen::Vector2d goalPose,double currq,double goalq);
// bool writeTrackResult(std::vector<geometry_msgs::Pose> posesvector,const std::string& filename);
// judge currend odom is near goal pose
// inline bool nearPose(Vector2d currPose,Vector2d goalPose,double currq,double goalq);
// double AngleMinDelta(double _start, double _goal);

/*可能有用的数据形式*****************************************************************************/
// reference: http://docs.ros.org/en/api/geometry_msgs/html/msg/Vector3.html
// geometry_msgs::Vector3 polytraj;    // 用于传递[x,y,q]多项式轨迹信息
// reference: http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PoseArray.html
// geometry_msgs::PoseArray polytraj;  // 用于传递[x,y,q]多项式轨迹信息

struct TrackSeg;
typedef TrackSeg* TrackSegPtr;

struct TrackSeg
{
    double duration;
    Eigen::Matrix<double, 3, 3> startState = Eigen::Matrix3Xd::Zero(3,3);
    Eigen::Matrix<double, 3, 3> endState = Eigen::Matrix3Xd::Zero(3,3);
    // std::vector<double> xcoeff;
    // std::vector<double> ycoeff;
    // std::vector<double> qcoeff;
    Eigen::VectorXd xcoeff;
    Eigen::VectorXd ycoeff;
    Eigen::VectorXd qcoeff;
    std::vector<double> pxtraj;
    std::vector<double> pytraj;
    std::vector<double> pqtraj;
    std::vector<double> vxtraj;
    std::vector<double> vytraj;
    std::vector<double> vqtraj;
};
/*用于轨迹跟踪的各种相关计算
 *
*/
class Tracking
{
    private:
    // Tracking 参数,persuit相关参数
    double _traj_time_interval; // 每个 tracking 周期的真实时间间隔
    double _time_interval;      // traj的时间离散间隔
    int _mpc_step_interval;  // MPC预测时间长度内有多少个时间步
    int _time_step_pursuit;  // 10Hz的期望轨迹周期 每个周期的期望轨迹步长(缩放与 _traj_time_interval / _time_interval有关)
    double _loop_rate;  // 10Hz的控制频率(默认)
    double _mpcHorizon; // MPC预测时间长度

    /// &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
    bool udp_switch;
    std::string local_ip = LOCAL_IP_ADDRESS;
    std::string dest_ip = DSET_IP_ADDRESS;
    int local_port = LOCAL_PORT;
    int dest_port = DEST_PORT;
    int udp_timeout;
    int loop_rate_hz;
    /// &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&

    // std::vector<double> pxtraj,pytraj,pqtraj; // 用于存储轨迹的px,py,pq for tracking
    // std::vector<double> vxtraj,vytraj,vqtraj; // 用于存储轨迹的vx,vy,vq for tracking
    std::vector<int> segtrajpoints; // 用于存储每段轨迹的离散点数
    // std::vector<double> timevector; // 用于存储每段轨迹的时间向量  这个时间向量没必要了,可以用下边的 tracksegsets
    // std::vector<TrackSeg> StatesSegSets; // 用于存储轨迹段集合 离散的状态轨迹去除,节省内存
    // 默认状态下存储的是整个搜索的轨迹，并将优化的部分插入替换

    // ROS 相关 Publisher/Subscriber/message
    int _nav_seq; // 控制轨迹系列计数
    ros::Publisher _nav_seq_pub; // 发布控制指令
    

    bool _nav_seq_vis_flag = true; // 是否可视化控制轨迹序列
    ros::Publisher _nav_seq_vis_pub; // 发布控制指令
    nav_msgs::Path _nav_seq_vis_msg;    // 控制轨迹序列msgs NMPC的TargetTrajectory

	bool BAIS_FLAG = false;	// 跟踪轨迹是否偏离(较大) 使用NMPC不会有非常大的偏差,这里是作为一个标志用于线段轨迹局部重规划

    public: // ##################################################################################################
    std::vector<TrackSeg> StatesSegSets;
    nav_msgs::Path _nav_seq_msg;    // 控制轨迹序列msgs NMPC的TargetTrajectory

    double _persuit_factor;         // 速度缩放因子,调整预瞄步长后缩放速度
    double _persuit_step_factor;    // 预测步长缩放因子,当前odometry在轨迹上点位判断的预瞄步长
    int max_bias_step;              // 最大局部偏差步长
    double _refer_vel,_refer_ome;   // 参考速度 线速度 角速度
    // Trajectory
	bool OBS_FLAG = false;	// 轨迹上是否有障碍物
	bool TROT_FLAG = false;	// 是否急刹车 轨迹上的障碍物距离机器人只有一个node的距离 
    bool CTRL_SWITCH = true;// 控制开关

    int _TO_SEG = 0; // 轨迹优化段

    //// real Trajectory &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
    clock_t _start_time,_end_time; // 用于计算轨迹跟踪的消耗时间
    double _track_time; // 用于计算轨迹跟踪的消耗时间
    std::vector<geometry_msgs::Pose> _real_poses;   // 用于存储真实轨迹
    std::vector<geometry_msgs::Twist> _real_twists; // 用于存储真实轨迹
    std::vector<nav_msgs::Odometry> _real_odoms;    // 用于存储真实轨迹
    std::string _realpose_address; // 用于存储真实轨迹的地址
    std::string _targetpose_address; // 用于存储真实轨迹的地址
    
    // trajectory param
    int _odom_cnt = 0;
    int _odom_cnt_th = 10;
    int _tracking_cnt = 0;
    std::vector<double> pxtraj,pytraj,pqtraj; // 用于存储轨迹的px,py,pq for tracking
    std::vector<double> vxtraj,vytraj,vqtraj; // 用于存储轨迹的vx,vy,vq for tracking
    // Tracking 有关的中间变量
    nav_msgs::Odometry::Ptr _currodometry; // 当前的odometry
    Eigen::Vector3d _current_odom;  //真实的当前位姿
    Eigen::Vector3d _local_odom;    //局部的当前位姿(处理 LeggedRobot 运动时的局部晃动)
    Eigen::Vector3d _goal_odom;     //真实的目标位姿
    Eigen::Vector3d _currPose,_pursuitPose,_goalPose; // 当前位姿,局部位姿,目标位姿 三个位姿在 TrackSegTraj 上
    // Eigen::Vector3d _currVel,_pursuitVel,_goalVel;    // 当前速度,局部速度,目标速度 三个速度在 TrackSegTraj 上
    // Eigen::Vector3d _currAcc,_pursuitAcc,_goalAcc;    // 当前加速度,局部加速度,目标加速度 三个加速度在 TrackSegTraj 上
    
    Eigen::Vector2i _current_index; // [0] segments [1] points
    Eigen::Vector2i _pursuit_index; // [0] segments [1] points
    int _serch_start_seg_index; // 轨迹搜索起始段
    int _opt_start_seg_index,_opt_end_seg_index;   // 轨迹优化起始段
    int _curr_time_step,_pursuit_time_step; // 当前时间步,局部 persuit 时间步
    int _curr_seg_points,_curr_traj_points; // 所有段的轨迹离散点数; 当前段的轨迹离散点数

	int _obs_seg_index,_obs_traj_index;	// 障碍物所在的轨迹段
    int _search_seg_index,_search_traj_index;	// 搜索轨迹所在的轨迹段

    // ##################################################################################################
    UDPBridge udp_bridge; // 用于发送控制指令的UDP通信

    void setParam(ros::NodeHandle& nh); // 设置参数
    void initPlanner(Eigen::Vector3d new_goal_odom);
    // void ReachGoal(); // 到达目标点
    bool replaceSegTraj(int seg_index,std::vector<TrackSeg> *tracksegsets); // 从重规划段插入新轨迹 (轨迹段数改变)
    bool insertSegTraj(int seg_index,std::vector<TrackSeg> *tracksegsets); // 替换优化的轨迹段 (轨迹段数不变)
    bool popOptSegTraj(int seg_index,std::vector<TrackSeg> *tracksegsets); // 从tracksegsets中弹出待优化轨迹
    bool OdometryIndex(Eigen::Vector3d odom); // 计算当前 odometry 所在的轨迹段和轨迹点, 如果跟踪紧密返回true，如果偏差较大返回false
    void NavSeqUpdate();    // 更新导航控制轨迹序列
    void NavSeqFixed(Eigen::Vector3d TargetPose);     // 固定导航控制轨迹序列
    void NavSeqPublish();   // 发布导航控制轨迹序列
	bool setObsTrajPoint(int obs_index); // 设置障碍物轨迹点

	int IndextoTraj(Eigen::Vector2i index); // 根据轨迹段和轨迹点计算轨迹序号
    Eigen::Vector2i TrajtoIndex(int traj_index); // 根据轨迹序号计算轨迹段和轨迹点

    // 辅助功能函数
    bool getCurrentState(Eigen::Vector3d* currPose,Eigen::Vector3d* currVel,Eigen::Vector3d* currAcc);
    bool getReplanState(Eigen::Vector3d* currPose,Eigen::Vector3d* currVel,Eigen::Vector3d* currAcc,Eigen::Vector3d *goalPose);
    bool getLocalState(Eigen::Vector3d* currPose,Eigen::Vector3d* currVel,Eigen::Vector3d* currAcc,Eigen::Vector3d *goalPose);
    bool isReachGoal(Eigen::Vector3d current_odom);

    void showTrackResult(); // 显示轨迹跟踪结果
    bool writeTrackResult(std::vector<nav_msgs::Odometry> posesvector,const std::string& filename);
    bool writeTrackTarget(const std::string& filename);
};

/***********************************************************************************************************************
 * @description: set planner params
 * @reference: 
 * @return {*}
 */
void Tracking::setParam(ros::NodeHandle& nh)
{
    nh.param("planner/time_interval", _time_interval, 0.01);
    nh.param("planner/loop_rate", _loop_rate, 10.0);
    nh.param("planner/mpcHorizon", _mpcHorizon, 1.0);
    // nh.param("planner/time_step_interval", _mpc_step_interval, int(10.0));
    // nh.param("planner/time_step_pursuit", _time_step_pursuit, int(5.0));
    
    // reference vel and ome 用于在局部偏差较大的时候规划局部轨迹 所以需要考虑 _persuit_factor
    nh.param("search/vel_factor",_refer_vel,2.0);
    nh.param("search/ome_factor", _refer_ome,1.57);

    nh.param("planner/nav_seq_vis", _nav_seq_vis_flag, true);
    nh.param("planner/persuit_factor", _persuit_factor, 1.0);
    nh.param("planner/persuit_step_factor", _persuit_step_factor, 1.0);
    nh.param("planner/max_bias_step", max_bias_step, AMX_BIAS_STEP);

    nh.param("planner/track_ctrl", CTRL_SWITCH, true);

    nh.param("planner/realpose_address", _realpose_address, std::string("~/realpose.csv"));
    nh.param("planner/targetpose_address", _targetpose_address, std::string("~/targetpose.csv"));

    // _persuit_factor 是 persuit 的缩放因子 根据 persuit 的时间步长来缩放速度
    if (_persuit_factor > 2.0) _persuit_factor = 2.0;
    _traj_time_interval = 1.0 / _loop_rate;                     // tracking 的周期
    _mpc_step_interval  = _mpcHorizon / _traj_time_interval;    // MPC预测时间长度内有多少tracking周期
    _time_step_pursuit  = _traj_time_interval / _time_interval; // 一个tracking周期内有多少个离散的状态点
    _time_step_pursuit  = _time_step_pursuit * _persuit_factor; // 根据 persuit_factor 缩放 persuit 的步长

    _nav_seq_pub        = nh.advertise<nav_msgs::Path>("/nav_seq",1);
    if (_nav_seq_vis_flag) _nav_seq_vis_pub    = nh.advertise<nav_msgs::Path>("/nav_seq_vis",1);

    ROS_INFO("[\033[34mTrackNode\033[0m]nav_seq_vis_pub   : %s",    _nav_seq_vis_flag  ? "true" : "false");
    ROS_INFO("[\033[34mTrackNode\033[0m]mpc_horizon       : %2.2f",  _mpcHorizon);
    ROS_INFO("[\033[34mTrackNode\033[0m]traj_time_interval: %2.4f",  _traj_time_interval);
    ROS_INFO("[\033[34mTrackNode\033[0m]mpc_step_interval : %4d",    _mpc_step_interval);
    ROS_INFO("[\033[34mTrackNode\033[0m]time_step_pursuit : %4d",    _time_step_pursuit);
    ROS_INFO("[\033[34mTrackNode\033[0m]persuit_factor    : %2.2f",  _persuit_factor);

    nh.param<bool>("udp_switch",udp_switch,false);
    nh.param<std::string>("local_ip_address", local_ip, LOCAL_IP_ADDRESS);
    nh.param<std::string>("dest_ip_address", dest_ip, DSET_IP_ADDRESS);
    nh.param<int>("local_port", local_port, LOCAL_PORT);
    nh.param<int>("dest_port", dest_port, DEST_PORT);
    nh.param<int>("timeout", udp_timeout, 2);
    nh.param<int>("loop_rate", loop_rate_hz, 10);

    if (udp_switch) {
        char ip[64] = {0};
        udp_bridge.setUDP(local_port,local_ip.c_str());
        printf(" Local IP: %s, Port: %d\n",
        inet_ntop(AF_INET, &udp_bridge.addr_local.sin_addr.s_addr, ip, sizeof(ip)),
        ntohs(udp_bridge.addr_local.sin_port));

        udp_bridge.setserverUDP(dest_port,dest_ip.c_str());
        printf(" Server IP: %s, Port: %d\n",
        inet_ntop(AF_INET, &udp_bridge.addr_server.sin_addr.s_addr, ip, sizeof(ip)),
        ntohs(udp_bridge.addr_server.sin_port));
    }
}

/***********************************************************************************************************************
 * @description: init planner params
 * @reference: 
 * @param {Vector3d} _new_goal_odom
 * @return {*}
 */
void Tracking::initPlanner(Eigen::Vector3d new_goal_odom)
{
    _goal_odom = new_goal_odom;
    _current_index = Eigen::Vector2i(0,0);
    _pursuit_index = Eigen::Vector2i(0,0);
    _serch_start_seg_index = 0; // 轨迹搜索起始点
    _opt_start_seg_index   = 0;   // 轨迹优化起始点
    _opt_end_seg_index     = _TO_SEG;   // 轨迹优化结束点
    
    _curr_time_step = 0;
    _pursuit_time_step = 0;
    _curr_seg_points  = 0;
    _curr_traj_points = 0;

    _obs_seg_index  = 0;
    _obs_traj_index = 0;
    _search_seg_index   = 0;
    _search_traj_index  = 0;

    _tracking_cnt = 0;
    if (!pxtraj.empty())
    {
        pxtraj.clear();
        pytraj.clear();
        pqtraj.clear();
        vxtraj.clear();
        vytraj.clear();
        vqtraj.clear();
        segtrajpoints.clear();
        StatesSegSets.clear();
    }
    if (!_real_poses.empty()) {
        _real_poses.clear();
    }
    if (!_real_odoms.empty()) {
        _real_odoms.clear();
    }
    _start_time = ros::Time::now().toNSec();
}
/***********************************************************************************************************************
 * @description: replace new trajectory from replan segment at seg_index
 * @reference: 
 * @param {int} seg_index
 * @param {vector<TrackSeg>} *tracksegsets : new trajectory
 * @return {bool} flag : if insert success return true else return false
 */
bool Tracking::replaceSegTraj(int seg_index,std::vector<TrackSeg> *tracksegsets)
{
    int _seg_num = tracksegsets->size(); // 替换轨迹的段数
    int _pre_traj_num = 0;
    int _new_traj_num = 0;
    int _erase_seg_num = 0;
    bool flag = true;
    // step 1: 计算当前 seg_index 前的轨迹点数
    if (seg_index > static_cast<int>(segtrajpoints.size())){
        ROS_ERROR("[\033[34mTrackNode\033[0m]seg_index is out of range");
        flag = false;
        return flag;
    }
    if (segtrajpoints.empty()){
        _pre_traj_num = 0;
    }
    else{
        for (int idx = 0; idx < seg_index && idx < static_cast<int>(segtrajpoints.size()); idx++){
            _pre_traj_num += segtrajpoints.at(idx);
        }
    }
    // step 2: 计算替换段的轨迹点数
    if (seg_index <= static_cast<int>(segtrajpoints.size())){ // 在原有轨迹的长度范围内计算总的点数
        
        for (int idx = seg_index; idx < static_cast<int>(segtrajpoints.size()); idx++){
            _new_traj_num += segtrajpoints.at(idx);
            _erase_seg_num ++;
        }
        // erase() 函数会改变容器的大小
        segtrajpoints.erase(segtrajpoints.begin() + seg_index, segtrajpoints.begin() + seg_index + _erase_seg_num);
        StatesSegSets.erase(StatesSegSets.begin() + seg_index, StatesSegSets.begin() + seg_index + _erase_seg_num);
    }
    // step 3: 将插入段的原本轨迹清除
    if (_pre_traj_num + _new_traj_num > static_cast<int>(pxtraj.size())){
        ROS_ERROR("[\033[34mTrackNode\033[0m]pre_traj_num + new_traj_num is out of range");
        flag = false;
        return flag;
    }
    if (_new_traj_num > 0) {
        pxtraj.erase(pxtraj.begin() + _pre_traj_num, pxtraj.begin() + _pre_traj_num + _new_traj_num);
        pytraj.erase(pytraj.begin() + _pre_traj_num, pytraj.begin() + _pre_traj_num + _new_traj_num);
        pqtraj.erase(pqtraj.begin() + _pre_traj_num, pqtraj.begin() + _pre_traj_num + _new_traj_num);
        vxtraj.erase(vxtraj.begin() + _pre_traj_num, vxtraj.begin() + _pre_traj_num + _new_traj_num);
        vytraj.erase(vytraj.begin() + _pre_traj_num, vytraj.begin() + _pre_traj_num + _new_traj_num);
        vqtraj.erase(vqtraj.begin() + _pre_traj_num, vqtraj.begin() + _pre_traj_num + _new_traj_num);
    }    
    // step 4: 将优化段的新轨迹插入跟踪轨迹
    std::vector<double> _new_pxtraj,_new_pytraj,_new_pqtraj;
    std::vector<double> _new_vxtraj,_new_vytraj,_new_vqtraj;
    std::vector<int> _new_segtrajpoints;
    std::vector<TrackSeg> _new_StatesSegSets;
    for (int idx = 0;idx < _seg_num;idx++)
    {
        TrackSeg _new_trackseg;
        _new_pxtraj.insert(_new_pxtraj.end(),tracksegsets->at(idx).pxtraj.begin(),tracksegsets->at(idx).pxtraj.end());
        _new_pytraj.insert(_new_pytraj.end(),tracksegsets->at(idx).pytraj.begin(),tracksegsets->at(idx).pytraj.end());
        _new_pqtraj.insert(_new_pqtraj.end(),tracksegsets->at(idx).pqtraj.begin(),tracksegsets->at(idx).pqtraj.end());
        _new_vxtraj.insert(_new_vxtraj.end(),tracksegsets->at(idx).vxtraj.begin(),tracksegsets->at(idx).vxtraj.end());
        _new_vytraj.insert(_new_vytraj.end(),tracksegsets->at(idx).vytraj.begin(),tracksegsets->at(idx).vytraj.end());
        _new_vqtraj.insert(_new_vqtraj.end(),tracksegsets->at(idx).vqtraj.begin(),tracksegsets->at(idx).vqtraj.end());
        _new_segtrajpoints.push_back(tracksegsets->at(idx).pxtraj.size());
        _new_trackseg.duration = tracksegsets->at(idx).duration;
        _new_trackseg.startState = tracksegsets->at(idx).startState;
        _new_trackseg.endState = tracksegsets->at(idx).endState;
        _new_trackseg.xcoeff = tracksegsets->at(idx).xcoeff;
        _new_trackseg.ycoeff = tracksegsets->at(idx).ycoeff;
        _new_trackseg.qcoeff = tracksegsets->at(idx).qcoeff;
        _new_StatesSegSets.push_back(_new_trackseg);
    }
    pxtraj.insert(pxtraj.begin() + _pre_traj_num , _new_pxtraj.begin(), _new_pxtraj.end());
    pytraj.insert(pytraj.begin() + _pre_traj_num , _new_pytraj.begin(), _new_pytraj.end());
    pqtraj.insert(pqtraj.begin() + _pre_traj_num , _new_pqtraj.begin(), _new_pqtraj.end());
    vxtraj.insert(vxtraj.begin() + _pre_traj_num , _new_vxtraj.begin(), _new_vxtraj.end());
    vytraj.insert(vytraj.begin() + _pre_traj_num , _new_vytraj.begin(), _new_vytraj.end());
    vqtraj.insert(vqtraj.begin() + _pre_traj_num , _new_vqtraj.begin(), _new_vqtraj.end());
    segtrajpoints.insert(segtrajpoints.begin() + seg_index , _new_segtrajpoints.begin(), _new_segtrajpoints.end());
    StatesSegSets.insert(StatesSegSets.begin() + seg_index , _new_StatesSegSets.begin(), _new_StatesSegSets.end());
    ROS_DEBUG("[\033[34mTrackNode\033[0m] insertSegTraj: seg_num: %d, traj_num: %ld",_seg_num,_new_pxtraj.size());
    return flag;
}

/***********************************************************************************************************************
 * @description: insert new trajectory from optimization segment at seg_index
 * @reference: 
 * @param {int} seg_index
 * @param {vector<TrackSeg>} *tracksegsets : new trajectory
 * @return {bool} flag : if insert success return true else return false
 */
bool Tracking::insertSegTraj(int seg_index,std::vector<TrackSeg> *tracksegsets)
{
    int _seg_num = tracksegsets->size(); // 插入轨迹的段数
    int _pre_traj_num = 0;
    int _new_traj_num = 0;
    int _erase_seg_num = 0;
    bool flag = true;
    // step 1: 计算当前 seg_index 前的轨迹点数
    if (seg_index > static_cast<int>(segtrajpoints.size())){
        ROS_ERROR("[\033[34mTrackNode\033[0m]seg_index is out of range");
        flag = false;
        return flag;
    }
    if (segtrajpoints.empty()){
        _pre_traj_num = 0;
    }
    else{
        for (int idx = 0; idx < seg_index && idx < static_cast<int>(segtrajpoints.size()); idx++){
            _pre_traj_num += segtrajpoints.at(idx);
        }
    }
    // step 2: 计算插入段的轨迹点数
    if (seg_index <= static_cast<int>(segtrajpoints.size())){ // 在原有轨迹的长度范围内计算总的点数
        
        for (int idx = seg_index; idx < _seg_num + seg_index && idx < static_cast<int>(segtrajpoints.size()); idx++){
            _new_traj_num += segtrajpoints.at(idx);
            _erase_seg_num ++;
        }
        // erase() 函数会改变容器的大小
        segtrajpoints.erase(segtrajpoints.begin() + seg_index, segtrajpoints.begin() + seg_index + _erase_seg_num);
        StatesSegSets.erase(StatesSegSets.begin() + seg_index, StatesSegSets.begin() + seg_index + _erase_seg_num);
    }
    // step 3: 将插入段的原本轨迹清除
    if (_pre_traj_num + _new_traj_num > static_cast<int>(pxtraj.size())){
        ROS_ERROR("[\033[34mTrackNode\033[0m]pre_traj_num + new_traj_num is out of range");
        flag = false;
        return flag;
    }
    if (_new_traj_num > 0) {
        pxtraj.erase(pxtraj.begin() + _pre_traj_num, pxtraj.begin() + _pre_traj_num + _new_traj_num);
        pytraj.erase(pytraj.begin() + _pre_traj_num, pytraj.begin() + _pre_traj_num + _new_traj_num);
        pqtraj.erase(pqtraj.begin() + _pre_traj_num, pqtraj.begin() + _pre_traj_num + _new_traj_num);
        vxtraj.erase(vxtraj.begin() + _pre_traj_num, vxtraj.begin() + _pre_traj_num + _new_traj_num);
        vytraj.erase(vytraj.begin() + _pre_traj_num, vytraj.begin() + _pre_traj_num + _new_traj_num);
        vqtraj.erase(vqtraj.begin() + _pre_traj_num, vqtraj.begin() + _pre_traj_num + _new_traj_num);
    }   
    // step 4: 将优化段的新轨迹插入跟踪轨迹
    std::vector<double> _new_pxtraj,_new_pytraj,_new_pqtraj;
    std::vector<double> _new_vxtraj,_new_vytraj,_new_vqtraj;
    std::vector<int> _new_segtrajpoints;
    std::vector<TrackSeg> _new_StatesSegSets;
    for (int idx = 0;idx < _seg_num;idx++)
    {
        TrackSeg _new_trackseg;
        _new_pxtraj.insert(_new_pxtraj.end(),tracksegsets->at(idx).pxtraj.begin(),tracksegsets->at(idx).pxtraj.end());
        _new_pytraj.insert(_new_pytraj.end(),tracksegsets->at(idx).pytraj.begin(),tracksegsets->at(idx).pytraj.end());
        _new_pqtraj.insert(_new_pqtraj.end(),tracksegsets->at(idx).pqtraj.begin(),tracksegsets->at(idx).pqtraj.end());
        _new_vxtraj.insert(_new_vxtraj.end(),tracksegsets->at(idx).vxtraj.begin(),tracksegsets->at(idx).vxtraj.end());
        _new_vytraj.insert(_new_vytraj.end(),tracksegsets->at(idx).vytraj.begin(),tracksegsets->at(idx).vytraj.end());
        _new_vqtraj.insert(_new_vqtraj.end(),tracksegsets->at(idx).vqtraj.begin(),tracksegsets->at(idx).vqtraj.end());
        _new_segtrajpoints.push_back(tracksegsets->at(idx).pxtraj.size());
        _new_trackseg.duration = tracksegsets->at(idx).duration;
        _new_trackseg.startState = tracksegsets->at(idx).startState;
        _new_trackseg.endState = tracksegsets->at(idx).endState;
        _new_trackseg.xcoeff = tracksegsets->at(idx).xcoeff;
        _new_trackseg.ycoeff = tracksegsets->at(idx).ycoeff;
        _new_trackseg.qcoeff = tracksegsets->at(idx).qcoeff;
        _new_StatesSegSets.push_back(_new_trackseg);
    }
    pxtraj.insert(pxtraj.begin() + _pre_traj_num , _new_pxtraj.begin(), _new_pxtraj.end());
    pytraj.insert(pytraj.begin() + _pre_traj_num , _new_pytraj.begin(), _new_pytraj.end());
    pqtraj.insert(pqtraj.begin() + _pre_traj_num , _new_pqtraj.begin(), _new_pqtraj.end());
    vxtraj.insert(vxtraj.begin() + _pre_traj_num , _new_vxtraj.begin(), _new_vxtraj.end());
    vytraj.insert(vytraj.begin() + _pre_traj_num , _new_vytraj.begin(), _new_vytraj.end());
    vqtraj.insert(vqtraj.begin() + _pre_traj_num , _new_vqtraj.begin(), _new_vqtraj.end());
    segtrajpoints.insert(segtrajpoints.begin() + seg_index , _new_segtrajpoints.begin(), _new_segtrajpoints.end());
    StatesSegSets.insert(StatesSegSets.begin() + seg_index , _new_StatesSegSets.begin(), _new_StatesSegSets.end());
    ROS_DEBUG("[\033[34mTrackNode\033[0m] insertSegTraj: seg_num: %d, traj_num: %ld",_seg_num,_new_pxtraj.size());
    return flag;
}

/***********************************************************************************************************************
 * @description: pop trajectory from tracksegsets at seg_index
 * @reference: 
 * @param {int} seg_index
 * @param {vector<TrackSeg>} *tracksegsets : new trajectory
 * @return {bool} flag : if pop success return true else return false
 */
bool Tracking::popOptSegTraj(int seg_index,std::vector<TrackSeg> *tracksegsets) {
    if (_TO_SEG < 0)
        return false;
    else if (_TO_SEG == 0)
        for (int idx = seg_index; idx < static_cast<int>(StatesSegSets.size()); idx ++)
            tracksegsets->push_back(StatesSegSets.at(idx));
    else if (_TO_SEG > 0) {
        for (int idx = seg_index; idx < seg_index + _TO_SEG && idx < static_cast<int>(StatesSegSets.size()); idx ++) 
            tracksegsets->push_back(StatesSegSets.at(idx));
        if ( _TO_SEG != static_cast<int>(tracksegsets->size())) {
            if (!tracksegsets->empty()) tracksegsets->clear();
            return false;
        }
    }
    ROS_DEBUG("[\033[34mTrackNode\033[0m] popOptSegTraj: seg_num: %ld",tracksegsets->size());
    return true;
}

/***********************************************************************************************************************
 * @description: 计算当前 odometry 所在的轨迹段和轨迹点, 转化为 curr_pose 如果跟踪紧密返回true，如果偏差较大返回false 
 * @reference: 
 * @param {Vector3d} _odompose
 * @return {bool} return_flag
 */
bool Tracking::OdometryIndex(Eigen::Vector3d odom)
{
    _current_odom = odom;  // 更新当前 odometry
    bool return_flag = true;   // 返回值
    // step 1: 计算当前 odometry 是否符合期望轨迹 正好在 persuit 轨迹上 证明跟踪紧密
    if (nearPose(_current_odom.head(2),_pursuitPose.head(2),_current_odom(2),_pursuitPose(2)))
    {
        _currPose = _pursuitPose;
        _current_index = _pursuit_index;
        _curr_time_step = _pursuit_time_step;
        _pursuit_time_step += _time_step_pursuit;
        if (_pursuit_time_step >= static_cast<int>(pqtraj.size()))
        {
            _pursuit_time_step = static_cast<int>(pqtraj.size()) - 1;
        }
        _pursuit_index = TrajtoIndex(_pursuit_time_step);

        //// 更新 opt_start_seg_index
        if (_pursuit_index(0) + NLOPT_BIAS> _opt_start_seg_index)
        {
            _opt_start_seg_index = _pursuit_index(0) + NLOPT_BIAS;
            _opt_end_seg_index  =   (_opt_start_seg_index + _TO_SEG > static_cast<int>(StatesSegSets.size())) ? 
                                    static_cast<int>(StatesSegSets.size()) : (_opt_start_seg_index + _TO_SEG);
        }

        _pursuitPose = Eigen::Vector3d(pxtraj.at(_pursuit_time_step), pytraj.at(_pursuit_time_step), pqtraj.at(_pursuit_time_step));
        return true;
    }
    // step 2: 如果不是紧密跟踪，计算当前 odometry 所在的轨迹段和轨迹点
    std::vector<double> dists;
    for (int idx = _pursuit_time_step - (_time_step_pursuit - PURSET_STEP_BACK_BIAS); idx <= _pursuit_time_step + _mpc_step_interval * _persuit_step_factor; idx++)
    {
        if (idx < 0)
        {
            dists.push_back(99.9);
            continue;
        }
        if (idx >= static_cast<int>(pqtraj.size()))
        {
            break;
        }
        // 这个距离只是xy的距离，没有考虑航向角
        double dist = sqrt(pow(odom(0) - pxtraj.at(idx), 2) + pow(odom(1) - pytraj.at(idx), 2));
        dists.push_back(dist);
    }
    // TODO: 这里存在Bug,如果最近的轨迹点在前面，就会导致跟踪的轨迹点后退
    // 还有一种情况是,_curr_time_step 的点距离odometry太远,这时候也需要重新设置轨迹
    // 找到最近的轨迹点 这里如果_curr_time_step > pqtraj.size()就设为最后一个点
    int min_idx = min_element(dists.begin(), dists.end()) - dists.begin();
    _curr_time_step = _pursuit_time_step - (_time_step_pursuit - PURSET_STEP_BACK_BIAS) + min_idx;
    if (_curr_time_step >= static_cast<int>(pqtraj.size()))
    {
        _curr_time_step = static_cast<int>(pqtraj.size()) - 1;
    }
    if (_curr_time_step < 0)
    {
        ROS_INFO("[\033[34mTrackNode\033[0m] _curr_time_step < 0");
        _curr_time_step = 0;
    }
    _current_index = TrajtoIndex(_curr_time_step);
    int trajpoint = pxtraj.size();
    _currPose = Eigen::Vector3d(pxtraj.at(_curr_time_step), pytraj.at(_curr_time_step), pqtraj.at(_curr_time_step));
    // step 3: 判断 _curr_time_step 是否在当前轨迹段内
    if (!nearPose(_current_odom.head(2),_currPose.head(2),_current_odom(2),_currPose(2)))
    {
        return_flag = false;
        double _x_get = _current_odom(0);
        double _y_get = _current_odom(1);
        double _q_get = _current_odom(2);
        // 如果不在当前轨迹段内，需要重新规划一个局部的轨迹段
        double _q_normalize = pqtraj.at(_curr_time_step);
        _q_normalize = angles::normalize_angle(_q_normalize);
        double _dist_xy_time = dists.at(min_idx) / _refer_vel;
        double _dist_q_time = std::abs(angles::shortest_angular_distance(_q_get, _q_normalize)) / _refer_ome;
        double _dist_time = std::max(_dist_xy_time, _dist_q_time);
        int _dist_time_step = std::ceil(_dist_time / _traj_time_interval);
        double _dist_x = pxtraj.at(_curr_time_step) - _x_get;
        double _dist_y = pytraj.at(_curr_time_step) - _y_get;
        double _dist_q = angles::shortest_angular_distance(_q_get, _q_normalize);
        double _x_vel = _dist_x / (_dist_time_step * _traj_time_interval) * _persuit_factor;
        double _y_vel = _dist_y / (_dist_time_step * _traj_time_interval) * _persuit_factor;
        double _q_vel = _dist_q / (_dist_time_step * _traj_time_interval) * _persuit_factor;
        geometry_msgs::PoseStamped nav_traj_msg;
        nav_traj_msg.header.frame_id = "planner";
        for (int idx = 1; idx < _dist_time_step + 1; idx++)
        {
            double _x_ref, _y_ref, _q_ref;
            _x_ref = _x_get + double(idx) / double(_dist_time_step) * _dist_x;
            _y_ref = _y_get + double(idx) / double(_dist_time_step) * _dist_y;
            _q_ref = _q_get + double(idx) / double(_dist_time_step) * _dist_q;
            // ####################################################################################
            nav_traj_msg.header.stamp = ros::Time::now();
            nav_traj_msg.pose.position.x = _x_ref;
            nav_traj_msg.pose.position.y = _y_ref;
            nav_traj_msg.pose.position.z = _q_ref;
            nav_traj_msg.pose.orientation.x = _x_vel;
            nav_traj_msg.pose.orientation.y = _y_vel;
            nav_traj_msg.pose.orientation.z = _q_vel;
            _nav_seq_msg.poses.push_back(nav_traj_msg);
            _nav_seq ++;
            // 如果超过了_time_step_interval步长,就不再添加轨迹点
            if (_nav_seq >= _mpc_step_interval)
                break; // 跳出当前for循环
        } // 注意这里的_nav_seq可能不够_time_step_interval步长 只填充了偏离轨迹的部分
    }
    if (static_cast<int>(_nav_seq_msg.poses.size()) < max_bias_step)  
    {
        _pursuit_time_step = _curr_time_step + _time_step_pursuit; // 这里判断一下误差 选择要不要推进跟踪点的前移
        if (_pursuit_time_step >= static_cast<int>(pqtraj.size()))
        {
            _pursuit_time_step = static_cast<int>(pqtraj.size()) - 1;
        }
        _pursuit_index = TrajtoIndex(_pursuit_time_step);
        _pursuitPose = Eigen::Vector3d(pxtraj.at(_pursuit_time_step), pytraj.at(_pursuit_time_step), pqtraj.at(_pursuit_time_step));
    }
    // 最后返回是否跟踪紧密
    return return_flag;
}

/***********************************************************************************************************************
 * @description: 更新导航控制轨迹序列 填补正常跟踪的轨迹点序列
 * @reference: 
 * @return {*}
 */
void Tracking::NavSeqUpdate()
{
    geometry_msgs::PoseStamped nav_traj_msg;
    nav_traj_msg.header.frame_id = "planner";

    // 如果轨迹点不够_time_step_interval步长,就从 pursuit_step 开始补充轨迹点
    if (_nav_seq < _mpc_step_interval)
    {
        int _temp_time_step = _pursuit_time_step;
        // 总之这个for循环可以把nav点补充到_time_step_interval步长
        for (int idx = 0; idx < _mpc_step_interval - _nav_seq; idx++)
        {
            if (_temp_time_step >= static_cast<int>(pqtraj.size()))
            {
                _temp_time_step = static_cast<int>(pqtraj.size()) - 1;
            }
            // ####################################################################################
            nav_traj_msg.header.stamp = ros::Time::now();
            nav_traj_msg.pose.position.x = pxtraj.at(_temp_time_step);
            nav_traj_msg.pose.position.y = pytraj.at(_temp_time_step);
            nav_traj_msg.pose.position.z = pqtraj.at(_temp_time_step);
            nav_traj_msg.pose.orientation.x = vxtraj.at(_temp_time_step) * _persuit_factor;
            nav_traj_msg.pose.orientation.y = vytraj.at(_temp_time_step) * _persuit_factor;
            nav_traj_msg.pose.orientation.z = vqtraj.at(_temp_time_step) * _persuit_factor;
            nav_traj_msg.pose.orientation.w = _traj_time_interval;
            _nav_seq_msg.poses.push_back(nav_traj_msg);

            _temp_time_step += _time_step_pursuit; // 应该从 pursuit_step 开始补充轨迹点
        }
    }
    
    // 第一个点为当前位姿 ####################################################################
    nav_traj_msg.header.stamp = ros::Time::now();
    nav_traj_msg.pose.position.x = _current_odom(0);
    nav_traj_msg.pose.position.y = _current_odom(1);
    nav_traj_msg.pose.position.z = _current_odom(2);
    nav_traj_msg.pose.orientation.x = 0;
    nav_traj_msg.pose.orientation.y = 0;
    nav_traj_msg.pose.orientation.z = 0;
    nav_traj_msg.pose.orientation.w = 0;
    _nav_seq_msg.poses.insert(_nav_seq_msg.poses.begin(),nav_traj_msg);
}

/***********************************************************************************************************************
 * @description: 固定导航控制轨迹序列 急刹车 or 原地踏步
 * @reference: 
 * @param {Vector3d} TargetPose
 * @return {*}
 */
void Tracking::NavSeqFixed(Eigen::Vector3d TargetPose)
{
    geometry_msgs::PoseStamped nav_traj_msg;
    nav_traj_msg.header.frame_id = "planner";
    if (!_nav_seq_msg.poses.empty())
        _nav_seq_msg.poses.clear();
    // 计算一个从 _current_odom 到 TargetPose 的轨迹 最快刹车? 感觉很难计算这个刹车轨迹
    // 如果轨迹点不够_time_step_interval步长,就从 pursuit_step 开始补充轨迹点
    // 总之这个for循环可以把 nav 点补充到_time_step_interval步长
    for (int idx = 0; idx < _mpc_step_interval; idx++)
    {
        // ####################################################################################
        nav_traj_msg.header.stamp = ros::Time::now();
        nav_traj_msg.pose.position.x = TargetPose(0);
        nav_traj_msg.pose.position.y = TargetPose(1);
        nav_traj_msg.pose.position.z = TargetPose(2);
        nav_traj_msg.pose.orientation.x = 0;
        nav_traj_msg.pose.orientation.y = 0;
        nav_traj_msg.pose.orientation.z = 0;
        nav_traj_msg.pose.orientation.w = _traj_time_interval;
        _nav_seq_msg.poses.push_back(nav_traj_msg);
    }
    // 第一个点为当前位姿 ####################################################################
    nav_traj_msg.header.stamp = ros::Time::now();
    nav_traj_msg.pose.position.x = _current_odom(0);
    nav_traj_msg.pose.position.y = _current_odom(1);
    nav_traj_msg.pose.position.z = _current_odom(2);
    nav_traj_msg.pose.orientation.x = 0;
    nav_traj_msg.pose.orientation.y = 0;
    nav_traj_msg.pose.orientation.z = 0;
    nav_traj_msg.pose.orientation.w = _traj_time_interval;
    _nav_seq_msg.poses.insert(_nav_seq_msg.poses.begin(),nav_traj_msg);
}

/***********************************************************************************************************************
 * @description: 发布导航控制轨迹序列 并清空导航控制序列话题
 * @reference: 
 * @return {*}
 */
void Tracking::NavSeqPublish()
{
    if (_nav_seq_vis_flag){
        geometry_msgs::PoseStamped nav_traj_vis_msg;
        nav_traj_vis_msg.header.frame_id = "planner";
        for (int idx = 0; idx < static_cast<int>(_nav_seq_msg.poses.size()); idx++){
            nav_traj_vis_msg.header.stamp = ros::Time::now();
            nav_traj_vis_msg.pose.position.x = _nav_seq_msg.poses.at(idx).pose.position.x;
            nav_traj_vis_msg.pose.position.y = _nav_seq_msg.poses.at(idx).pose.position.y;
            nav_traj_vis_msg.pose.position.z = _currodometry->pose.pose.position.z; // 这里的z就是里程计的高度
            // nav_traj_vis_msg.pose.position.z = STAND_HIGH; // 这里的z就是Unitree-A1 的默认站立高度
            nav_traj_vis_msg.pose.orientation = tf::createQuaternionMsgFromYaw(_nav_seq_msg.poses.at(idx).pose.position.z);
            _nav_seq_vis_msg.poses.push_back(nav_traj_vis_msg);
        }
        _nav_seq_vis_msg.header.frame_id = "planner";
        _nav_seq_vis_msg.header.stamp = ros::Time::now();
        _nav_seq_vis_pub.publish(_nav_seq_vis_msg);
        _nav_seq_vis_msg.poses.clear();
    }
    for (int idx=0;idx<_nav_seq_msg.poses.size();idx++)
    {
        // V_I 转化到 V_B
        // double _dot_q = _nav_seq_msg.poses.at(idx).pose.position.z;
        // double _xvel = _nav_seq_msg.poses.at(idx).pose.orientation.x;
        // double _yvel = _nav_seq_msg.poses.at(idx).pose.orientation.y;
        // _nav_seq_msg.poses.at(idx).pose.orientation.x = _yvel * sin(_dot_q) + _xvel * cos(_dot_q);
        // _nav_seq_msg.poses.at(idx).pose.orientation.y = _yvel * cos(_dot_q) - _xvel * sin(_dot_q);

        // 速度根据状态估计做缩放 
        _nav_seq_msg.poses.at(idx).pose.orientation.x = _nav_seq_msg.poses.at(idx).pose.orientation.x * VEL_SCALE;
        _nav_seq_msg.poses.at(idx).pose.orientation.y = _nav_seq_msg.poses.at(idx).pose.orientation.y * VEL_SCALE;
        _nav_seq_msg.poses.at(idx).pose.orientation.z = _nav_seq_msg.poses.at(idx).pose.orientation.z * ANG_SCALE;
    }
    _nav_seq_msg.header.frame_id = "planner";
    _nav_seq_msg.header.stamp = ros::Time::now();
    if (udp_switch) {
        udp_bridge.resetsend();
        udp_bridge.setROSNavseq(_nav_seq_msg);
        udp_bridge.setHeaderandBail();
        udp_bridge.send();
    }
    if (CTRL_SWITCH)
        _nav_seq_pub.publish(_nav_seq_msg);
    _nav_seq_msg.poses.clear();
    _nav_seq = 0;
}

/***********************************************************************************************************************
 * @description: 获取当前状态
 * @reference: 
 * @param {Vector3d} *currPose
 * @param {Vector3d} *currVel
 * @param {Vector3d} *currAcc
 * @return {bool} 是否成功获取当前状态 false 当前无跟踪轨迹,默认初始0状态 true 当前在跟踪轨迹,返回当前轨迹点的状态
 */
bool Tracking::getCurrentState(Eigen::Vector3d* currPose,Eigen::Vector3d* currVel,Eigen::Vector3d* currAcc)
{
    bool flag = false;
    if ( _tracking_cnt == 0) { // 没有跟踪轨迹
        (*currPose) = _current_odom;
        currVel->setZero();
        currAcc->setZero();
        flag = false;
    }
    // else{
    //     if (pxtraj.empty() || pytraj.empty() || pqtraj.empty())
    //         (*currPose) = _current_odom;
    //     else
    //         (*currPose) = _currPose;        
    //     if (vxtraj.empty() || vytraj.empty() || vqtraj.empty())
    //         currVel->setZero();
    //     else
    //         (*currVel)  = Eigen::Vector3d(vxtraj.at(_curr_time_step),vytraj.at(_curr_time_step),vqtraj.at(_curr_time_step));

    //     (*currAcc)  = Eigen::Vector3d(0,0,0);
    //     flag = true;
    // }
    else{
        (*currPose) = _current_odom;
        currVel->setZero();
        currAcc->setZero();
        flag = true;
    }
    return flag;
}

/***********************************************************************************************************************
 * @description: 获取重规划点的状态
 * @reference: 
 * @param {Vector3d} *currPose
 * @param {Vector3d} *currVel
 * @param {Vector3d} *currAcc
 * @return {bool} 是否成功获取重规划点状态 false 当前无跟踪轨迹,默认初始0状态 true 当前在跟踪轨迹,返回当前轨迹点的状态
 */
bool Tracking::getReplanState(Eigen::Vector3d* currPose,Eigen::Vector3d* currVel,Eigen::Vector3d* currAcc,Eigen::Vector3d *goalPose)
{
    bool flag = false;
    if ( _search_traj_index < static_cast<int>(pxtraj.size())){
        (*currPose) = Eigen::Vector3d(pxtraj.at(_search_traj_index),pytraj.at(_search_traj_index),pqtraj.at(_search_traj_index));
        // (*currVel)  = Eigen::Vector3d(vxtraj.at(_search_traj_index),vytraj.at(_search_traj_index),vqtraj.at(_search_traj_index));
        (*currVel)  = Eigen::Vector3d(0,0,0);
        (*currAcc)  = Eigen::Vector3d(0,0,0);
        (*goalPose) = _goal_odom;
        flag = true;
    }
    return flag;
}

/***********************************************************************************************************************
 * @description: 初次重规划无效后就取距离 robot 更近的点来规划
 * @reference: 
 * @param {Vector3d*} currPose
 * @param {Vector3d*} currVel
 * @param {Vector3d*} currAcc
 * @param {Vector3d} *goalPose
 * @return {bool}
 */
bool Tracking::getLocalState(Eigen::Vector3d* currPose,Eigen::Vector3d* currVel,Eigen::Vector3d* currAcc,Eigen::Vector3d *goalPose)
{
    bool flag = false;
    int local_index = _current_index[0] + 1; // 当前tracking段 + 1
    local_index = IndextoTraj(Eigen::Vector2i(local_index,0));
    if ( local_index < static_cast<int>(pxtraj.size())){
        (*currPose) = Eigen::Vector3d(pxtraj.at(local_index),pytraj.at(local_index),pqtraj.at(local_index));
        (*currVel)  = Eigen::Vector3d(vxtraj.at(local_index),vytraj.at(local_index),vqtraj.at(local_index));
        (*currAcc)  = Eigen::Vector3d(0,0,0);
        (*goalPose) = _goal_odom;
        flag = true;
    }
    return flag;
}
/***********************************************************************************************************************
 * @description: 设置障碍物轨迹点,并返回判断是否危险
 * @reference: 
 * @param {int} obs_index
 * @return {bool} danger_flag : true 危险/急刹车 false 安全/继续跟踪
 */
bool Tracking::setObsTrajPoint(int obs_index)
{
	OBS_FLAG = true;
	Eigen::Vector2i _obsIndex = TrajtoIndex(obs_index);
	_obs_traj_index = obs_index;
	_obs_seg_index  = _obsIndex(0);
    ////&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
    // 处理重规划起始点
    _search_seg_index = _obs_seg_index - 1;
    if (_search_seg_index - _current_index(0) > REPLAN_BIAS)
        _search_seg_index = _current_index(0) + REPLAN_BIAS;
    ////&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
    _search_traj_index = IndextoTraj(Eigen::Vector2i(_search_seg_index,0));
	if (_obs_seg_index <= _current_index(0)) {// 障碍物挨的太近了
		TROT_FLAG = true;
	}else{
		TROT_FLAG = false;
	}
    return TROT_FLAG;
}

/***********************************************************************************************************************
 * @description: 判断是否到达目标点
 * @reference: 
 * @param {Vector3d} current_odom
 * @return {bool} true 到达目标点 false 未到达目标点
 */
bool Tracking::isReachGoal(Eigen::Vector3d current_odom) // TODO:
{
    // 这里的_goalPose 是经过 lazykinoprm 重规划后的目标点(_goal_odom 是原始目标点 可能在障碍物里面)
    if (nearPose(current_odom.head(2),_goalPose.head(2),current_odom(2),_goalPose(2))){
        return true;
    }
    else{
        return false;
    }
}

// ######################################################################################################################
// pravite 辅助函数

/***********************************************************************************************************************
 * @description: 根据轨迹段和轨迹点计算轨迹序号 
 * @reference: 
 * @param {Vector2i} index
 * @return {int} _traj_index
 */
inline int Tracking::IndextoTraj(Eigen::Vector2i index)
{
    int _traj_index = 0;
    for (int idx = 0; idx < index(0); idx++)
    {
        _traj_index += segtrajpoints.at(idx);
    }
    _traj_index += index(1);
    return _traj_index;
}

/***********************************************************************************************************************
 * @description: 根据轨迹序号计算轨迹段和轨迹点
 * @reference: 
 * @param {int} traj_index
 * @return {Eigen::Vector2i} (_seg_index,_point_index)
 */
inline Eigen::Vector2i Tracking::TrajtoIndex(int traj_index)
{
    int _seg_index = 0;
    int _traj_points = 0;
    for (int idx = 0; idx < static_cast<int>(segtrajpoints.size()); idx++)
    {
        _traj_points += segtrajpoints.at(idx);
        if (traj_index < _traj_points)
        {
            _seg_index = idx;
            break;
        }
    }
    _traj_points -= segtrajpoints.at(_seg_index);
    int _point_index = traj_index - _traj_points;
    return Eigen::Vector2i(_seg_index,_point_index);
}

void Tracking::showTrackResult() {
    std::cout << "######################## Tracking Result ########################"<< std::endl;
    std::cout << "tracking count: " << _tracking_cnt << std::endl;
    _tracking_cnt = 0;
    // std::cout << "real odometry size: " << _real_poses.size() << std::endl;
    std::cout << "real odometry size: " << _real_odoms.size() << std::endl;
    double _track_time = (double)(_end_time - _start_time) / 10e6; // 用于计算轨迹跟踪的消耗时间
    std::cout << "tracking time: " << _track_time << "ms" << std::endl;
    if (!_real_odoms.empty()) {
        writeTrackResult(_real_odoms,_realpose_address);
    }
    if (!pxtraj.empty()) {
        writeTrackTarget(_targetpose_address);
    }
}

//判断两个位姿是否相近
inline bool nearPose(Eigen::Vector2d currPose,Eigen::Vector2d goalPose,double currq,double goalq)
{
    bool falg=false;
    double dist = (currPose - goalPose).norm();
    double qerr = fabs(currq - goalq);
    if(dist < DIST_XY && qerr < DIST_Q) falg=true;
    else falg=false;
    return falg;
}

// bool writeTrackResult(std::vector<geometry_msgs::Pose> posesvector,const std::string& filename){
bool Tracking::writeTrackResult(std::vector<nav_msgs::Odometry> odomsvector,const std::string& filename){
    std::ofstream file(filename);
    if (file) {
        file.close();
        file.open(filename, std::ios_base::out | std::ios_base::trunc);
    }

    for (int i = 0; i < odomsvector.size(); i++) {
        geometry_msgs::Pose pose = odomsvector[i].pose.pose;
        geometry_msgs::Twist twist = odomsvector[i].twist.twist;
        
        file    << pose.position.x << "," 
                << pose.position.y << "," 
                << pose.position.z << "," 
                << pose.orientation.x << "," 
                << pose.orientation.y << "," 
                << pose.orientation.z << "," 
                << pose.orientation.w << "," 
                << twist.linear.x << ","
                << twist.linear.y << ","
                << twist.linear.z << ","
                << twist.angular.x << ","
                << twist.angular.y << ","
                << twist.angular.z << std::endl;
    }
    file.close();
    return true;
}

bool Tracking::writeTrackTarget(const std::string& filename){
    std::ofstream file(filename);
    if (file) {
        file.close();
        file.open(filename, std::ios_base::out | std::ios_base::trunc);
    }

    for (int i = 0; i < pxtraj.size(); i++) {
        
        file    << pxtraj[i] << "," 
                << pytraj[i] << "," 
                << pqtraj[i] << "," 
                << vxtraj[i] << "," 
                << vytraj[i] << "," 
                << vqtraj[i] << std::endl;
    }
    file.close();
    return true;
}

#endif