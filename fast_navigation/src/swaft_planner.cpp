/*
 * @Author: wentao zhang && zwt190315@163.com
 * @Date: 2023-06-23
 * @LastEditTime: 2023-06-29
 * @Description: swaft planner for fast real time navigation 
 * @reference: 
 * 
 * Copyright (c) 2023 by wentao zhang, All Rights Reserved. 
 */

#include <fast_navigation/navigation.hpp>

using namespace std;
using namespace Eigen;
using namespace cv;

// robot visulaization param
//unitree A1 robot size
#define DEFAULT_HIGH 0.30f
#define DEFAULT_WIDTH 0.30f
#define DEFAULT_LENGTH 0.40f

#define Q_WEIGHT 0.01f

// visualization_msgs::Marker 可视化的透明度
#define RGB_ALPHA 0.6f

// ros related global variables
ros::Subscriber _map_sub, _pts_sub, _odom_sub;  // 订阅地图;导航点;里程计
int _map_sub_cnt = 5, _odom_sub_cnt = 5;        // 计数器 判断是否正常sub
#define SUB_CNT_TH 10                           // 订阅计数器阈值 Hz = 10

ros::Publisher _nav_seq_vis_pub;    // 导航点可视化
ros::Publisher _obsmap_img_pub,_sdfmap_img_pub; // 地图可视化
ros::Publisher _ptraj_vis_pub,_vtraj_vis_pub;   // 轨迹可视化
ros::Publisher _path_vis_pub,_robot_vis_pub;    // 导航可视化

////####################################################################################
// node parameters
double _local_width;    // local map width
double _loop_rate;      // node loop rate
double _time_interval;  // time interval for trajectory
// visualization switch
bool _vis_Robot = false;
bool _vis_sample = false;
bool _nav_seq_vis_flag = true;
double _resolution;

////####################################################################################
// trajectory tracking GLOBAL FLAGS
bool _HAS_MAP   = false;    // 是否有地图
bool _HAS_PATH  = false;    // 搜索到路径，每次 lazykinoprm 搜索到路径后，就会置为true; Obstacle check infeasible 会置为false
bool _NEW_PATH  = true;     // 是否是新的路径 重新设置目标点后,就是新的路径
bool _FIRST_ODOM= true;     // 是否是第一次接收到里程计信息 
bool _REACH_GOAL= false;    // 是否到达目标点
bool _TRACKING  = false;    // 是否正在跟踪轨迹 安全检查不通过就会置为false 原地急刹车trot

////####################################################################################
// trajectory optimization parameters 
int _OPT_SEG = 0;           // 优化轨迹的段数 这个参数根据 sample 的 grid size 和 reference velocity 来确定 
                            // 小于0就不优化 | 等于0就是全优化 | 大于0就是根据采样区间和参考速度计算优化长度
double _optHorizon = 1.5;   // 优化轨迹的时间长度
#define OPT_FACTOR 1.5f

nav_msgs::Odometry::ConstPtr currodometry;
// visualization robot switch 在ros循环中可视化 

// Trajectory Tracking Manager
Tracking tracking;
// Lazy Kinodynamic Path Searching
LazyKinoPRM lazykinoPRM;

// Trajectory Struct
vector<TrackSeg> searchTraj;    // 存储搜索到的轨迹
vector<TrackSeg> optimalTraj;   // 存储优化的轨迹

// 回调函数 ############################################################################################################
// 目标点回调函数，进行路径规划
void rcvWaypointsCallback(const nav_msgs::Path & wp);
// 点云回调函数，负责设置障碍地图并更新SDF地图
void rcvPointCloudCallBack(const sensor_msgs::PointCloud2 & pointcloud_map);
// 里程计回调函数，负责设置机器人位置;进行轨迹跟踪
void rcvOdomCallBack(const nav_msgs::Odometry::ConstPtr& msg);

// 关键功能函数 ############################################################################################################
// search trajectory checking
bool TrackTrajCheck();
// 搜索到的 tracksegs 从 lazykinoprm 类 push 到 searchTraj 列表
void SearchSegPush();
// 优化后的 tracksegs 从 optimalTraj 列表 pop 到 tracking 类中
void OptimalSegPop();
// 待优化的 tracksegs 从 searchTraj 列表 push 到 optimalTraj 列表
void OptimalSegPush();
// 可视化函数 ############################################################################################################
// visulization search trajectory
void visSearchTraj();
// visulization tracking trajectory
// bool visTrackingTraj();
// visulization robot 
void visRobot(const nav_msgs::Odometry::ConstPtr& msg);
// visulization position trajectory
void visPtraj();
// visulization velocity trajectory
void visVtraj();
// save trajectory as .csv file
void csvPtraj();

int main(int argc, char** argv)
{
    ros::init(argc, argv, "swaft_planner_node");
    ros::NodeHandle nh("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);

    // 地图和导航点订阅
    _map_sub  = nh.subscribe( "map",       1, rcvPointCloudCallBack );
    _pts_sub  = nh.subscribe( "waypoints", 1, rcvWaypointsCallback );
    _odom_sub = nh.subscribe( "odom",      1, rcvOdomCallBack );

    // 路径和地图可视化
    _path_vis_pub             = nh.advertise<visualization_msgs::MarkerArray>("planner_path_vis",1);
    // _obsmap_img_pub           = nh.advertise<sensor_msgs::Image>("obs_map_img",1);
    _sdfmap_img_pub           = nh.advertise<sensor_msgs::Image>("sdf_map_img",1);
    _robot_vis_pub            = nh.advertise<visualization_msgs::MarkerArray>("robot_vis",1);
    // _cmd_vel_pub              = nh.advertise<geometry_msgs::Twist>("/cmd_vel",1);
    _nav_seq_vis_pub          = nh.advertise<nav_msgs::Path>("/nav_seq_vis",1);

    // 轨迹可视化
    _ptraj_vis_pub            = nh.advertise<geometry_msgs::PoseArray>("/ptraj_vis",1);
    _vtraj_vis_pub            = nh.advertise<geometry_msgs::PoseArray>("/vtraj_vis",1);

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

    // 读取参数 ################################################################################################################
    // 地图相关参数
    nh.param("map/local_width", _local_width, 2.0);
    nh.param("map/resolution", _resolution, 0.05);
    // 轨迹跟踪相关参数
    nh.param("planner/loop_rate", _loop_rate, 1.0);
    nh.param("planner/vis_Robot", _vis_Robot, false);
    nh.param("planner/vis_sample", _vis_sample, false);
    nh.param("planner/nav_seq_vis", _nav_seq_vis_flag, true);
    nh.param("planner/time_interval", _time_interval, 0.1); 
    nh.param("planner/optHorizon", _optHorizon, 1.5); 

    // 计算优化相关的参数 ################################################################################################################
    double _opt_seg = 0,xy_sample_size_ = 0;
    nh.param("planner/opt_seg", _opt_seg, 0.0);
    nh.param("search/xy_sample_size", xy_sample_size_, 0.4);
    _OPT_SEG = int(_opt_seg);
    if (_OPT_SEG > 0){ // 根据采样区间和参考速度计算优化长度
        double sample_grid_time = xy_sample_size_ / tracking._refer_vel;
        _OPT_SEG = int(_optHorizon / sample_grid_time * OPT_FACTOR); // 优化轨迹的段数
    }

    // 轨迹跟踪相关参数
    // ROS_INFO("[\033[34mPlanNode\033[0m]planner/time_horizon: %2.4f",        tracking._time_horizon);
    // ROS_INFO("[\033[34mPlanNode\033[0m]planner/traj_time_interval: %2.4f",  tracking._traj_time_interval);
    // ROS_INFO("[\033[34mPlanNode\033[0m]planner/time_step_interval: %d",     tracking._time_step_interval);
    // ROS_INFO("[\033[34mPlanNode\033[0m]planner/time_step_pursuit: %d",      tracking._time_step_pursuit);
    // ROS_INFO("[\033[34mPlanNode\033[0m]planner/vel_factor: %2.4f",          tracking._vel_factor);

    ros::Rate rate(_loop_rate);
    bool status = ros::ok();
    while(status) 
    {
        ros::spinOnce();
        // step 轨迹的安全性检查 ############################################################################################################
        // 检查接下来需要跟踪的轨迹是否安全
        bool feasible = TrackTrajCheck();
        if (tracking.TROT_FLAG){ // 障碍物距离太近,需要急刹车再重新规划
            _TRACKING = false;
            tracking.NavSeqFixed(tracking._pursuitPose);
            tracking.NavSeqPublish();
            ROS_INFO("[\033[31mPlanNode\033[0m]: track traj is not feasible");
        }
        if (!feasible){  // 轨迹上有障碍物
            ROS_INFO("[\033[31mPlanNode\033[0m]: track traj is not feasible");
            _HAS_PATH = false;
            Vector3d goalPose,currPose,currVel,currAcc,zeros_pt(0,0,0);
            tracking.getReplanState(&currPose,&currVel,&currAcc,&goalPose);
            _HAS_PATH = !lazykinoPRM.search(currPose,currVel,currAcc,goalPose,zeros_pt,zeros_pt);
            if (_HAS_PATH){ // 重新规划成功
                tracking._goalPose = lazykinoPRM._goal_pose;
                SearchSegPush();
                tracking.insertSegTraj(tracking._search_seg_index,&searchTraj);
            }
            visSearchTraj();
            // 可视化轨迹后再清空搜索列表
            ROS_INFO("[\033[34mPlanNode\033[0m]: reset planning done");
            lazykinoPRM.reset();
        }
        // step 轨迹的优化 ##########################################################################################################

        // step 可视化 ############################################################################################################
        // 可视化机器人和感知范围
        if (_vis_Robot){
            visRobot(currodometry);
            _vis_Robot=false;
        }


        status = ros::ok();
        rate.sleep();
    }

    return 0;
}

/************************************************************************************************************
 * @description: 期望路标点回调函数,进行路径规划,收到新的路标点后,舍弃原本目标,立马重新规划
 * @reference: 
 * @param {Path &} wp 期望路标点
 * @return {*}
 */
void rcvWaypointsCallback(const nav_msgs::Path & wp)
{
    double yaw_rad,yaw_deg;
    Vector3d goalPose,currPose,currVel,currAcc,zeros_pt(0,0,0);
    goalPose(0) = wp.poses[0].pose.position.x;
    goalPose(1) = wp.poses[0].pose.position.y;
    goalPose(2) = tf::getYaw(wp.poses[0].pose.orientation); // use tf2 library to get yaw from quaternion
    yaw_rad = goalPose(2);
    yaw_deg = yaw_rad * 180 / M_PI;
    tracking.getCurrentState(&currPose,&currVel,&currAcc);

    ROS_INFO("[\033[34mPlanNode\033[0m] receive the planning target");
    ROS_INFO("[\033[34mPlanNode\033[0m]:currPose:  %2.2f, %2.2f, %2.2f", currPose(0), currPose(1), currPose(2));
    ROS_INFO("[\033[34mPlanNode\033[0m]:currVel:   %2.2f, %2.2f, %2.2f", currVel(0), currVel(1), currVel(2));
    ROS_INFO("[\033[34mPlanNode\033[0m]:currAcc:   %2.2f, %2.2f, %2.2f", currAcc(0), currAcc(1), currAcc(2));
    ROS_INFO("[\033[34mTarget\033[0m]:position:    %2.2f, %2.2f, %2.2f", wp.poses[0].pose.position.x, wp.poses[0].pose.position.y, wp.poses[0].pose.position.z);
    ROS_INFO("[\033[34mTarget\033[0m]:orientation: %2.2f, %2.2f, %2.2f, %2.2f", wp.poses[0].pose.orientation.x, wp.poses[0].pose.orientation.y, wp.poses[0].pose.orientation.z, wp.poses[0].pose.orientation.w);
    ROS_INFO("[\033[34mTarget\033[0m]: x:%2.2f, y:%2.2f, q:%2.2f", goalPose(0), goalPose(1), yaw_deg);

    // ####################################################################################
    // ROS_INFO("[\033[34mTarget\033[0m]:yaw angle %f", yaw_deg);
    _HAS_PATH = !lazykinoPRM.search(currPose,currVel,currAcc,goalPose,zeros_pt,zeros_pt);

    if (_HAS_PATH)
    {
        _NEW_PATH = true;
        tracking.initPlanner(goalPose);
        tracking._goalPose = lazykinoPRM._goal_pose;
        SearchSegPush();
        tracking.insertSegTraj(0,&searchTraj);
        // double trajectory_length = 0, trajectory_time = 0;
        // if (!searchTraj.empty())
        //     searchTraj.clear();

        // for (int idx = 0; idx < int(lazykinoPRM.pathstateSets.size()); idx++)
        // {
        //     TrackSeg _trackseg;
        //     _trackseg.pxtraj = lazykinoPRM.pathstateSets[idx].polytraj('p', 'x', _time_interval);
        //     _trackseg.pytraj = lazykinoPRM.pathstateSets[idx].polytraj('p', 'y', _time_interval);
        //     _trackseg.pqtraj = lazykinoPRM.pathstateSets[idx].polytraj('p', 'q', _time_interval);
        //     _trackseg.vxtraj = lazykinoPRM.pathstateSets[idx].polytraj('v', 'x', _time_interval);
        //     _trackseg.vytraj = lazykinoPRM.pathstateSets[idx].polytraj('v', 'y', _time_interval);
        //     _trackseg.vqtraj = lazykinoPRM.pathstateSets[idx].polytraj('v', 'q', _time_interval);
        //     _trackseg.startState << lazykinoPRM.pathstateSets[idx].ParePosition , 
        //                             lazykinoPRM.pathstateSets[idx].PareVelocity , 
        //                             lazykinoPRM.pathstateSets[idx].PareAcceleration;
        //     _trackseg.endState   << lazykinoPRM.pathstateSets[idx].Position ,
        //                             lazykinoPRM.pathstateSets[idx].Velocity ,
        //                             lazykinoPRM.pathstateSets[idx].Acceleration;
        //     _trackseg.xcoeff = lazykinoPRM.pathstateSets[idx].xpcoeff;
        //     _trackseg.ycoeff = lazykinoPRM.pathstateSets[idx].ypcoeff;
        //     _trackseg.qcoeff = lazykinoPRM.pathstateSets[idx].qpcoeff;
        //     _trackseg.duration = lazykinoPRM.pathstateSets[idx].referT;

        //     searchTraj.push_back(_trackseg);
        //     trajectory_length += lazykinoPRM.pathstateSets[idx].trajectory_length;
        //     trajectory_time += lazykinoPRM.pathstateSets[idx].referT;
        // }
        // ROS_INFO("[\033[32mPlanNode\033[0m]: search trajectory length: %2.4f", trajectory_length);
        // ROS_INFO("[\033[32mPlanNode\033[0m]: search trajectory time: %2.4f", trajectory_time);
        // save trajectory as .CSV
        // csvPtraj();
    }
    // ####################################################################################
    visSearchTraj();
    // 可视化轨迹后再清空搜索列表
    ROS_INFO("[\033[34mPlanNode\033[0m]: reset planning done");
    lazykinoPRM.reset();
}

// 点云回调函数，负责设置障碍物
void rcvPointCloudCallBack(const sensor_msgs::PointCloud2 & pointcloud_map)
{   
    _map_sub_cnt--;
    if (_map_sub_cnt < 0)
        _map_sub_cnt=5;;

    pcl::PointCloud<pcl::PointXYZ> cloud;
    sensor_msgs::PointCloud2 map_vis;

    pcl::fromROSMsg(pointcloud_map, cloud);

    ROS_DEBUG("[Node]cloud.points.size() = %d", (int)cloud.points.size());
    
    if( (int)cloud.points.size() == 0 ) { // 如果点云为空，就不进行地图更新，但是可能是正常的(没有障碍物)
        ROS_INFO("[\033[33m !!!!! rcvPCL\033 none points !!!!![0m]:");
        return;
    }
    lazykinoPRM.updataObsMap(cloud);
    cv::Mat* obsimg = lazykinoPRM.getObsMap();
    // cv::Mat obsptr = *lazykinoPRM.obs_map;
    ROS_DEBUG("[Node]obsimg.size() = %d, %d", obsimg->rows, obsimg->cols);
    // 这里好像没必要打印obsimg,因为sdf_img已经包含了obsimg的信息
    // ROS_DEBUG("[\033[34mNode\033[0m]obsptr.size() = %d, %d", obsptr.rows, obsptr.cols);
    // ROS_DEBUG("obsimg.at(%d,%d) = %d", 100,100,obsimg.at<uchar>(100,100));
    // sensor_msgs::ImagePtr image_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", *obsimg).toImageMsg();
    // _obsmap_img_pub.publish(image_msg);
    cv::Mat* sdf_map_ptr = lazykinoPRM.getSdfMap();
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
    sensor_msgs::ImagePtr image_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", merge_img).toImageMsg();
    image_msg->header.stamp = ros::Time::now();
    _sdfmap_img_pub.publish(image_msg);
    _HAS_MAP = true;

    // ####################################################################################
    //// 在点云回调函数中进行轨迹检查感觉比较合理呀,可以减少里程计回调中,轨迹跟踪的计算量
    //// 还是在主循环中进行轨迹检查比较合理呀,中断回调里面最好不要执行太多的操作
}

// 里程计回调函数，负责设置机器人位置;进行轨迹跟踪
void rcvOdomCallBack(const nav_msgs::Odometry::ConstPtr& msg)
{
    // 发布机器人位姿
    currodometry = msg;
    _vis_Robot = true;

    // 轨迹控制 跟踪位置/速度
        // if( _HAS_PATH == false ) return;
    nav_msgs::Path _nav_seq_msg;
    double _x_get, _y_get, _q_get;
    // double _x_ref,_y_ref,_q_ref;
    double x_vel, y_vel, q_vel;
    // double _x_err,_y_err,_q_err;
    _x_get = msg->pose.pose.position.x;
    _y_get = msg->pose.pose.position.y;
    _q_get = tf::getYaw(msg->pose.pose.orientation); // use tf2 library to get yaw from quaternion
    // _q_get 的范围是 -pi ~ pi

    _currPose << _x_get, _y_get, _q_get;

    
    _nav_seq_msg.header.frame_id = "world";
    _nav_seq_msg.header.stamp = ros::Time::now();

    // 机器人当前位置与目标位置的距离和角度差 控制轨迹跟踪的第一个点是当前点
    // 用于locomotion controller中计算相对参考轨迹
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header.stamp = ros::Time::now();
    pose_msg.header.frame_id = "world";
    pose_msg.pose.position.x = _x_get;
    pose_msg.pose.position.y = _y_get;
    pose_msg.pose.position.z = DEFAULT_HIGH;
    pose_msg.pose.orientation = tf::createQuaternionMsgFromYaw(_q_get);
    _nav_seq_vis_msg.poses.push_back(pose_msg);
    // ####################################################################################
    geometry_msgs::PoseStamped nav_traj_msg;
    nav_traj_msg.header.stamp = ros::Time::now();
    nav_traj_msg.header.frame_id = "world";
    nav_traj_msg.pose.position.x = _x_get;
    nav_traj_msg.pose.position.y = _y_get;
    nav_traj_msg.pose.position.z = _q_get;
    nav_traj_msg.pose.orientation.x = 0;
    nav_traj_msg.pose.orientation.y = 0;
    nav_traj_msg.pose.orientation.z = 0;
    _nav_seq_msg.poses.push_back(nav_traj_msg);

    uint8_t _nav_seq = 0;

    // 初始的_odom_cnt_th 个里程计信息不使用
    if (_odom_cnt < _odom_cnt_th)
    {
        _odom_cnt++;
        return;
    }
    // 如果是第一次接收到里程计信息，就将当前位置设置为起始位置
    if (_FIRST_ODOM)
    {
        _localPose = _currPose;
        ROS_DEBUG("[PlannerNode]:_localPose = %f, %f, %f", _localPose(0), _localPose(1), _localPose(2));
        _FIRST_ODOM = false;
    }

    if (_HAS_PATH)
    {
        // 如果已经到达目标点，就不再进行轨迹跟踪 
        if (nearPose(Vector2d(_x_get, _y_get), Vector2d(_goalPose(0), _goalPose(1)), _q_get, _goalPose(2)))
        {
            ROS_INFO("[\033[32mPlannerNode\033[0m]: goal reached!");
            ROS_INFO("[\033[32mPlannerNode\033[0m]: tacking time = %2.4f", double(_tracking_cnt)*_traj_time_interval);
            _HAS_PATH = false;
            _NEW_PATH = true;
            _localPose = _goalPose;
            _tracking_cnt = 0;
            return;
        }
        _tracking_cnt ++;
        // 如果跟踪的是新的轨迹，就重新设置轨迹
        if (_NEW_PATH)
        {
            _NEW_PATH = false;
            _curr_time_step = 0;
            _goal_time_step = _time_step_pursuit;
            ROS_INFO("[\033[34mPlannerNode\033[0m]: new path traj.size() = %d", xtraj.size());
        }
        else
        {
            vector<double> dists;
            // 在_time_step_interval步长范围内，搜索当前/odometry最近的轨迹点 注意这里往前搜索了 一定的时间步,有可能出现比当前时间步还小的时间步
            // 另外还会产生问题,就是整个的搜索范围太小,导致搜索到的curr_time_step过于滞后,按理来说_time_step_pursuit是根据时间的离散关系计算的
            // 10hz的频率正常跟踪应该在一个合适的范围内呀
            // Bug: 在跟踪结束的部分,老是会一直跳到这里,一直打印idx >= pqtraj.size()的信息
            for (int idx = _goal_time_step - _time_step_pursuit / 2; idx <= _goal_time_step + _time_step_interval; idx++)
            {
                if (idx < 0)
                {
                    ROS_INFO("[\033[33mPlannerNode\033[0m]: idx = %d", idx);
                    continue;
                }
                if (idx >= pqtraj.size())
                {
                    // ROS_INFO("[\033[33mPlannerNode\033[0m]: idx >=  pqtraj.size() = %d", idx);
                    // ROS_INFO("[\033[33mPlannerNode\033[0m]: currPose:[%2.4f %2.4f %2.4f]",_x_get, _y_get, _q_get);
                    // ROS_INFO("[\033[33mPlannerNode\033[0m]: goalPose:[%2.4f %2.4f %2.4f]",_goalPose(0), _goalPose(1), _goalPose(2));
                    // ROS_INFO("[\033[33mPlannerNode\033[0m]: trajPose:[%2.4f %2.4f %2.4f]",pxtraj.at(_curr_time_step), pytraj.at(_curr_time_step), pqtraj.at(_curr_time_step));
                    break;
                }
                // 这个距离只是xy的距离，没有考虑航向角
                double dist = sqrt(pow(_x_get - pxtraj.at(idx), 2) + pow(_y_get - pytraj.at(idx), 2));
                dists.push_back(dist);
            }
            // TODO: 这里存在Bug,如果最近的轨迹点在前面，就会导致跟踪的轨迹点后退
            // 还有一种情况是,_curr_time_step 的点距离odometry太远,这时候也需要重新设置轨迹
            // 找到最近的轨迹点 这里如果_curr_time_step > pqtraj.size()就设为最后一个点
            int min_idx = min_element(dists.begin(), dists.end()) - dists.begin();
            _curr_time_step = _goal_time_step - _time_step_pursuit / 2 + min_idx;
            // 这里_curr_time_step 是不可能有>= pqtraj.size()的情况？
            if (_curr_time_step >= pqtraj.size())
            {
                 _curr_time_step = pqtraj.size() - 1;
                 ROS_WARN("[\033[33mPlannerNode\033[0m]: _curr_time_step >= pqtraj.size() = %d", _curr_time_step);
            }
            // 如果当前的odometry距离轨迹点很远,就重新设置这段轨迹
            // 这部分老出问题,问题表现在,odometry已经超前了，但是又触犯了!nearPose的条件，导致机器人卡在这里前后跳动
            // bool flag = nearPose(Vector2d(_x_get, _y_get), Vector2d(pxtraj.at(_curr_time_step), pytraj.at(_curr_time_step)), _q_get, pqtraj.at(_curr_time_step));
            // if (!flag)
            if (!nearPose(Vector2d(_x_get, _y_get), Vector2d(pxtraj.at(_curr_time_step), pytraj.at(_curr_time_step)), _q_get, pqtraj.at(_curr_time_step)))
            {
                _goal_time_step = _curr_time_step + _time_step_pursuit;
                if (_goal_time_step >= pqtraj.size())
                    _goal_time_step = pqtraj.size() - 1;
                double _q_normalize = pqtraj.at(_curr_time_step);
                _q_normalize = angles::normalize_angle(_q_normalize);
                double _dist_xy_time = dists.at(min_idx) / _refer_vel;
                double _dist_q_time = abs(angles::shortest_angular_distance(_q_get, _q_normalize)) / _refer_ome;
                double _dist_time = max(_dist_xy_time, _dist_q_time);
                int _dist_time_step = ceil(_dist_time / _traj_time_interval);
                double _dist_x = pxtraj.at(_curr_time_step) - _x_get;
                double _dist_y = pytraj.at(_curr_time_step) - _y_get;
                double _dist_q = angles::shortest_angular_distance(_q_get, _q_normalize);
                double _x_vel = _dist_x / (_dist_time_step * _traj_time_interval);
                double _y_vel = _dist_y / (_dist_time_step * _traj_time_interval);
                double _q_vel = _dist_q / (_dist_time_step * _traj_time_interval);
                for (int idx = 1; idx < _dist_time_step + 1; idx++)
                {
                    double _x_ref, _y_ref, _q_ref;
                    _x_ref = _x_get + double(idx) / double(_dist_time_step) * _dist_x;
                    _y_ref = _y_get + double(idx) / double(_dist_time_step) * _dist_y;
                    _q_ref = _q_get + double(idx) / double(_dist_time_step) * _dist_q;
                    // _q_ref = angles::normalize_angle(_q_ref); // 规范化角度
                    geometry_msgs::PoseStamped pose_msg;
                    pose_msg.header.stamp = ros::Time::now();
                    pose_msg.header.frame_id = "world";
                    pose_msg.pose.position.x = _x_ref;
                    pose_msg.pose.position.y = _y_ref;
                    pose_msg.pose.position.z = DEFAULT_HIGH;
                    pose_msg.pose.orientation = tf::createQuaternionMsgFromYaw(_q_ref);
                    _nav_seq_vis_msg.poses.push_back(pose_msg);
                    // ####################################################################################
                    geometry_msgs::PoseStamped nav_traj_msg;
                    nav_traj_msg.header.stamp = ros::Time::now();
                    nav_traj_msg.header.frame_id = "world";
                    nav_traj_msg.pose.position.x = _x_ref;
                    nav_traj_msg.pose.position.y = _y_ref;
                    nav_traj_msg.pose.position.z = _q_ref;
                    nav_traj_msg.pose.orientation.x = _x_vel;
                    nav_traj_msg.pose.orientation.y = _y_vel;
                    nav_traj_msg.pose.orientation.z = _q_vel;
                    _nav_seq_msg.poses.push_back(nav_traj_msg);
                    _nav_seq ++;
                    // 如果超过了_time_step_interval步长,就不再添加轨迹点
                    if (_nav_seq >= _time_step_interval)
                        break; // 跳出当前for循环
                } // 注意这里的_nav_seq可能不够_time_step_interval步长 只填充了偏离轨迹的部分
                // ####################################################################################
                // ROS_INFO("[\033[33mPlannerNode\033[0m]: odom is far from traj time:%2.4f, delta_q:%2.4f", _dist_time,_dist_q);
                // ROS_INFO("[\033[33mPlannerNode\033[0m]: local dist x: %2.4f,y: %2.4f,q: %2.4f", _dist_x, _dist_y, _dist_q);
                // ROS_INFO("[\033[33mPlannerNode\033[0m]: local vel x: %2.4f,y: %2.4f,q: %2.4f", _x_vel, _y_vel, _q_vel);
                // ROS_INFO("[\033[33mPlannerNode\033[0m]: currPose:[%2.4f %2.4f %2.4f]",_x_get, _y_get, _q_get);
                // ROS_INFO("[\033[33mPlannerNode\033[0m]: trajVel :[%2.4f %2.4f %2.4f]",_x_vel, _y_vel, _q_vel);
                // ROS_INFO("[\033[33mPlannerNode\033[0m]: trajPose:[%2.4f %2.4f %2.4f]",pxtraj.at(_curr_time_step), pytraj.at(_curr_time_step), _q_normalize);
            }
            else // 如果当前的odometry距离轨迹点很近,就正常跟踪
            {
                // 从当前的轨迹点开始,向后推_time_horizon时间的轨迹,下一时刻的目标点是_time_step_pursuit步长的点
                _goal_time_step = _curr_time_step + _time_step_pursuit;
                if (_goal_time_step >= pqtraj.size())
                    _goal_time_step = pqtraj.size() - 1;
            } 
        }     // 这里的else是if (_NEW_PATH)的else
        // ####################################################################################
        // 从当前的轨迹点开始,填充轨迹点,直到_time_step_interval步长
        if (_nav_seq < _time_step_interval)
        {
            for (int idx = 1; idx < _time_step_interval + 1; idx++)
            {
                if (_curr_time_step + idx * _time_step_pursuit >= pqtraj.size())
                    break;
                // 注意这里的_nav_seq可能不够_time_step_interval步长
                double _q_ref = pqtraj.at(_curr_time_step + idx * _time_step_pursuit);
                // _q_ref = angles::normalize_angle(_q_ref); // 规范化角度
                geometry_msgs::PoseStamped pose_msg;
                pose_msg.header.stamp = ros::Time::now();
                pose_msg.header.frame_id = "world";
                pose_msg.pose.position.x = pxtraj.at(_curr_time_step + idx * _time_step_pursuit);
                pose_msg.pose.position.y = pytraj.at(_curr_time_step + idx * _time_step_pursuit);
                pose_msg.pose.position.z = DEFAULT_HIGH;
                pose_msg.pose.orientation = tf::createQuaternionMsgFromYaw(_q_ref);
                _nav_seq_vis_msg.poses.push_back(pose_msg);
                // ####################################################################################
                geometry_msgs::PoseStamped nav_traj_msg;
                nav_traj_msg.header.stamp = ros::Time::now();
                nav_traj_msg.header.frame_id = "world";
                nav_traj_msg.pose.position.x = pxtraj.at(_curr_time_step + idx * _time_step_pursuit);
                nav_traj_msg.pose.position.y = pytraj.at(_curr_time_step + idx * _time_step_pursuit);
                nav_traj_msg.pose.position.z = _q_ref;
                nav_traj_msg.pose.orientation.x = vxtraj.at(_curr_time_step + idx * _time_step_pursuit) * _vel_factor;
                nav_traj_msg.pose.orientation.y = vytraj.at(_curr_time_step + idx * _time_step_pursuit) * _vel_factor;
                nav_traj_msg.pose.orientation.z = vqtraj.at(_curr_time_step + idx * _time_step_pursuit) * _vel_factor;
                _nav_seq_msg.poses.push_back(nav_traj_msg);
                _nav_seq++;
                // 如果超过了_time_step_interval步长,就不再添加轨迹点
                if (_nav_seq >= _time_step_interval)
                    break; // 跳出当前for循环
            }// 注意这里的_nav_seq可能不够_time_step_interval步长,因为当前点可能已经非常接近终点了
            // ####################################################################################
            // 如果当前点已经非常接近终点了,剩下的Traj就直接填充终点
            for (int idx = _nav_seq;idx < _time_step_interval; idx++)
            {
                double _q_ref = pqtraj.back();
                // _q_ref = angles::normalize_angle(_q_ref); // 规范化角度
                geometry_msgs::PoseStamped pose_msg;
                pose_msg.header.stamp = ros::Time::now();
                pose_msg.header.frame_id = "world";
                pose_msg.pose.position.x = pxtraj.back();
                pose_msg.pose.position.y = pytraj.back();
                pose_msg.pose.position.z = DEFAULT_HIGH;
                pose_msg.pose.orientation = tf::createQuaternionMsgFromYaw(_q_ref);
                _nav_seq_vis_msg.poses.push_back(pose_msg);
                // ####################################################################################
                geometry_msgs::PoseStamped nav_traj_msg;
                nav_traj_msg.header.stamp = ros::Time::now();
                nav_traj_msg.header.frame_id = "world";
                nav_traj_msg.pose.position.x = pxtraj.back();
                nav_traj_msg.pose.position.y = pytraj.back();
                nav_traj_msg.pose.position.z = _q_ref;
                nav_traj_msg.pose.orientation.x = 0;
                nav_traj_msg.pose.orientation.y = 0;
                nav_traj_msg.pose.orientation.z = 0;
                _nav_seq_msg.poses.push_back(nav_traj_msg);
                _nav_seq++;
                // 如果超过了_time_step_interval步长,就不再添加轨迹点
                if (_nav_seq >= _time_step_interval)
                    break; // 跳出当前for循环
            }
        }//
        // ####################################################################################
        double _q_ref = pqtraj.at(_goal_time_step);
        _q_ref = angles::normalize_angle(_q_ref); // 规范化角度
        _localPose << pxtraj.at(_goal_time_step), pytraj.at(_goal_time_step), _q_ref;
    }
    else // 这个else是if (_HAS_PATH)的else
    {
        geometry_msgs::PoseStamped pose_msg;
        pose_msg.header.stamp = ros::Time::now();
        pose_msg.header.frame_id = "world";
        pose_msg.pose.position.x = _localPose(0);
        pose_msg.pose.position.y = _localPose(1);
        pose_msg.pose.position.z = DEFAULT_HIGH;
        pose_msg.pose.orientation = tf::createQuaternionMsgFromYaw(_localPose(2));
        // ####################################################################################
        geometry_msgs::PoseStamped nav_traj_msg;
        nav_traj_msg.header.stamp = ros::Time::now();
        nav_traj_msg.header.frame_id = "world";
        nav_traj_msg.pose.position.x = _localPose(0);
        nav_traj_msg.pose.position.y = _localPose(1);
        nav_traj_msg.pose.position.z = _localPose(2);
        nav_traj_msg.pose.orientation.x = 0;
        nav_traj_msg.pose.orientation.y = 0;
        nav_traj_msg.pose.orientation.z = 0;
        for (int idx = _nav_seq; idx < _time_step_interval; idx++)
        {
            _nav_seq ++;
            _nav_seq_vis_msg.poses.push_back(pose_msg);
            _nav_seq_msg.poses.push_back(nav_traj_msg);
        }
    }
    // 如果当前位置已经在目标位置附近，就停止
    _nav_seq_pub.publish(_nav_seq_msg);
}


/*******************************************************************************************
 * @description: 当前点开始检查轨迹是否无障碍
 * @reference: 
 * @return {bool} feasible true if feasible false if not
 */
bool TrackTrajCheck() // TODO: add BAIS_FLAG for trajectory check NMPC 跟踪效果很好感觉没啥必要
{
    int curr_ser_traj = tracking._curr_time_step;
    bool feasible = lazykinoPRM.LongTrajCheck(&tracking.pxtraj,&tracking.pytraj,&curr_ser_traj);
    if (!feasible) {
        tracking.setObsTrajPoint(curr_ser_traj);
    }
    return feasible;
}

void SearchSegPush()
{
    double trajectory_length = 0, trajectory_time = 0;
    if (!searchTraj.empty())
    searchTraj.clear();
    for (int idx = 0; idx < int(lazykinoPRM.pathstateSets.size()); idx++)
    {
        TrackSeg _trackseg;
        _trackseg.pxtraj = lazykinoPRM.pathstateSets[idx].polytraj('p', 'x', _time_interval);
        _trackseg.pytraj = lazykinoPRM.pathstateSets[idx].polytraj('p', 'y', _time_interval);
        _trackseg.pqtraj = lazykinoPRM.pathstateSets[idx].polytraj('p', 'q', _time_interval);
        _trackseg.vxtraj = lazykinoPRM.pathstateSets[idx].polytraj('v', 'x', _time_interval);
        _trackseg.vytraj = lazykinoPRM.pathstateSets[idx].polytraj('v', 'y', _time_interval);
        _trackseg.vqtraj = lazykinoPRM.pathstateSets[idx].polytraj('v', 'q', _time_interval);
        _trackseg.startState << lazykinoPRM.pathstateSets[idx].ParePosition , 
                                lazykinoPRM.pathstateSets[idx].PareVelocity , 
                                lazykinoPRM.pathstateSets[idx].PareAcceleration;
        _trackseg.endState   << lazykinoPRM.pathstateSets[idx].Position ,
                                lazykinoPRM.pathstateSets[idx].Velocity ,
                                lazykinoPRM.pathstateSets[idx].Acceleration;
        _trackseg.xcoeff = lazykinoPRM.pathstateSets[idx].xpcoeff;
        _trackseg.ycoeff = lazykinoPRM.pathstateSets[idx].ypcoeff;
        _trackseg.qcoeff = lazykinoPRM.pathstateSets[idx].qpcoeff;
        _trackseg.duration = lazykinoPRM.pathstateSets[idx].referT;

        searchTraj.push_back(_trackseg);
        trajectory_length += lazykinoPRM.pathstateSets[idx].trajectory_length;
        trajectory_time += lazykinoPRM.pathstateSets[idx].referT;
    }
    ROS_INFO("[\033[32mPlanNode\033[0m]: search trajectory length: %2.4f", trajectory_length);
    ROS_INFO("[\033[32mPlanNode\033[0m]: search trajectory time: %2.4f", trajectory_time);
}

void OptimalSegPop()
{

}

//########################################################################################
// 可视化函数


// void visPtraj()
// {
//     nav_msgs::Path path;
//     path.header.frame_id = "world";
//     path.header.stamp = ros::Time::now();
//     path.poses.resize(pqtraj.size()/5);
//     for (int i = 0; i < pqtraj.size(); i=i+5)
//     {
//         path.poses[i].pose.position.x = pxtraj.at(i);
//         path.poses[i].pose.position.y = pytraj.at(i);
//         path.poses[i].pose.position.z = DEFAULT_HIGH;
//         path.poses[i].pose.orientation = tf::createQuaternionMsgFromYaw(pqtraj.at(i));
//     }
//     _ptraj_vis_pub.publish(path);
// }

// void visVtraj()
// {

// }


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
    ROS_INFO("[\033[35mNodeCSV\033[0m] datas.size() = %d", tracking.pxtraj.size());
    for (int i = 0; i < tracking.pqtraj.size(); i++)
    {
        outfile << tracking.pxtraj.at(i) << "," << tracking.pytraj.at(i) << "," << tracking.pqtraj.at(i) << ","
             << tracking.vxtraj.at(i) << "," << tracking.vytraj.at(i) << "," << tracking.vqtraj.at(i) << endl;
    }
    outfile.close();
    csv_record_num++;
}


/*******************************************************************************************
 * @description:  可视化搜索到的: 轨迹 采样点 终点
 * @reference: 
 * @return {*}
 */
void visSearchTraj()
{
    double _resolution = 0.02;
    visualization_msgs::MarkerArray  LineArray;
    visualization_msgs::Marker       Line;
    visualization_msgs::Marker       Spheres;
    visualization_msgs::Marker       ESpheres;
    visualization_msgs::Marker       OSpheres;
    visualization_msgs::Marker       GSphere;

    // Path State Sets Visualization
    Spheres.header.frame_id =  Line.header.frame_id = "world";
    Spheres.header.stamp    =  Line.header.stamp    = ros::Time::now();
    Spheres.ns              =  Line.ns              = "planner_node/TraLibrary";
    Spheres.action          =  Line.action          = visualization_msgs::Marker::ADD;
    // Search Tree Visualization
    ESpheres.header.frame_id =  OSpheres.header.frame_id = "world";
    ESpheres.header.stamp    =  OSpheres.header.stamp    = ros::Time::now();
    ESpheres.ns              =  OSpheres.ns              = "planner_node/TraLibrary";
    ESpheres.action          =  OSpheres.action          = visualization_msgs::Marker::ADD;
    // Goal Point Visualization
    GSphere.header.frame_id = "world";
    GSphere.header.stamp    = ros::Time::now();
    GSphere.ns              = "planner_node/TraLibrary";
    GSphere.action          = visualization_msgs::Marker::ADD;

    Line.pose.orientation.w = 1.0;
    Line.type            = visualization_msgs::Marker::LINE_STRIP;
    Line.scale.x         = _resolution;

    Line.color.r         = 0.0;
    Line.color.g         = 0.0;
    Line.color.b         = 1.0;
    Line.color.a         = 1.0;

    Spheres.pose.orientation.w = 1.0;
    Spheres.type            = visualization_msgs::Marker::SPHERE_LIST;
    Spheres.scale.x         = _resolution*5;
    Spheres.scale.y         = _resolution*5;
    Spheres.scale.z         = _resolution*5;

    Spheres.color.a         = 0.8;
    Spheres.color.r         = 0.0;
    Spheres.color.g         = 0.0;
    Spheres.color.b         = 1.0;

    ESpheres.pose.orientation.w = 1.0;
    ESpheres.type            = visualization_msgs::Marker::SPHERE_LIST;
    ESpheres.scale.x         = _resolution*5;
    ESpheres.scale.y         = _resolution*5;
    ESpheres.scale.z         = _resolution*5;

    ESpheres.color.a         = 0.6;
    ESpheres.color.r         = 1.0;
    ESpheres.color.g         = 0.0;
    ESpheres.color.b         = 0.0;

    OSpheres.pose.orientation.w = 1.0;
    OSpheres.type            = visualization_msgs::Marker::SPHERE_LIST;
    OSpheres.scale.x         = _resolution*5;
    OSpheres.scale.y         = _resolution*5;
    OSpheres.scale.z         = _resolution*5;

    OSpheres.color.a         = 0.6;
    OSpheres.color.r         = 0.0;
    OSpheres.color.g         = 1.0;
    OSpheres.color.b         = 0.0;
    
    GSphere.pose.orientation.w = 1.0;
    GSphere.type            = visualization_msgs::Marker::SPHERE;
    GSphere.scale.x         = _resolution*2;
    GSphere.scale.y         = _resolution*2;
    GSphere.scale.z         = _resolution*2;

    GSphere.color.a         = 1;
    GSphere.color.r         = 0.0;
    GSphere.color.g         = 0.0;
    GSphere.color.b         = 0.0;
    
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
    if (lazykinoPRM.pathstateSets.size() == 0)
    {
        ROS_INFO("[\033[34mvisTraLibrary\033[0m]No path found!");
        // return;
    }
    else
    {
        for (int idx = 0; idx < int(lazykinoPRM.pathstateSets.size()); idx++)
        {
            vector<double> xtraj, ytraj;
            xtraj = lazykinoPRM.pathstateSets[idx].polytraj('p', 'x', _time_interval);
            ytraj = lazykinoPRM.pathstateSets[idx].polytraj('p', 'y', _time_interval);
            for (int i = 0; i < int(xtraj.size()); i++)
            {
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

        for (int idx = 0; idx < int(lazykinoPRM.pathstateSets.size()); idx++)
        {
            Vector3d coord = lazykinoPRM.pathstateSets[idx].Position;
            pt.x = coord(0);
            pt.y = coord(1);
            pt.z = DEFAULT_HIGH;
            ROS_INFO("[\033[34mvisTraLibrary\033[0m]Position = [%f, %f, %f]", coord(0), coord(1), coord(2));
            Spheres.points.push_back(pt);
        }
        LineArray.markers.push_back(Spheres);
        ++marker_id;
    }

    // #######################################################################################
    // Search Tree Visualization
    if (lazykinoPRM.astaropenlist.nodestateSets.size() == 0)
    {
        ROS_INFO("[\033[34mvisTraLibrary\033[0m]No OpenList found!");
    }
    else // node of openlist
    {
        OSpheres.id = marker_id;
        ++marker_id;
        ESpheres.id = marker_id;
        ++marker_id;
        for (int idx = 0; idx < int(lazykinoPRM.astaropenlist.nodestateSets.size()); idx++)
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
        ROS_DEBUG("[visSearchTraj]LineArray.size() = %d", (int)LineArray.markers.size());
        ROS_DEBUG("[visSearchTraj]marker_id = %d", marker_id);
    }
    _path_vis_pub.publish(LineArray);
}

/*******************************************************************************************
 * @description:  可视化 robot : body | local window | persuit point
 * @reference: 
 * @param {ConstPtr&} msg
 * @return {*}
 */
void visRobot(const nav_msgs::Odometry::ConstPtr& msg)
{   
    // 获取机器人当前位姿
    geometry_msgs::Pose pose = msg->pose.pose;
    double _resolution = 0.02;
    visualization_msgs::MarkerArray  LineArray;
    visualization_msgs::Marker       Line;
    visualization_msgs::Marker       Body;
    visualization_msgs::Marker       Spheres;
    
    Body.header.frame_id =  Line.header.frame_id = "world";
    Body.header.stamp    =  Line.header.stamp    = ros::Time::now();
    Body.ns              =  Line.ns              = "planner_node/visRobot";
    Body.action          =  Line.action          = visualization_msgs::Marker::ADD;

    Spheres.header.frame_id = "world";
    Spheres.header.stamp    = ros::Time::now();
    Spheres.ns              = "planner_node/visRobot";
    Spheres.action          = visualization_msgs::Marker::ADD;

    Line.id              = 1;
    Line.pose.orientation.w = 1.0;
    Line.type            = visualization_msgs::Marker::LINE_STRIP;
    Line.scale.x         = _resolution;

    Line.color.r         = 0.0;
    Line.color.g         = 0.0;
    Line.color.b         = 1.0;
    Line.color.a         = 1.0;

    Body.id              = 2;
    Body.pose.orientation.w = 1.0;
    Body.type            = visualization_msgs::Marker::CUBE;
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
    Spheres.scale.x         = _resolution*5;
    Spheres.scale.y         = _resolution*5;
    Spheres.scale.z         = _resolution*5;

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

    LineArray.markers.push_back(Spheres);

    _robot_vis_pub.publish(LineArray);
}

/********************************************************************************/