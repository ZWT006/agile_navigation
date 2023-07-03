/*
 * @Author: wentao zhang && zwt190315@163.com
 * @Date: 2023-06-23
 * @LastEditTime: 2023-07-03
 * @Description: swaft planner for fast real time navigation 
 * @reference: 
 * 
 * Copyright (c) 2023 by wentao zhang, All Rights Reserved. 
 */

// #include <>

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

// ros::Publisher _nav_seq_vis_pub;    // 导航点可视化
ros::Publisher _obsmap_img_pub,_sdfmap_img_pub; // 地图可视化
ros::Publisher _ptraj_vis_pub,_vtraj_vis_pub;   // 轨迹可视化
ros::Publisher _path_vis_pub,_robot_vis_pub;    // 导航可视化

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
bool _nav_seq_vis_flag = true;
double _vis_resolution = 0.02; // 可视化的分辨率
double _resolution;     // obs map 分辨率
// 轨迹安全性检查长度分辨率
double _DIST_RES = 0.04; // 4cm

////####################################################################################
// trajectory tracking GLOBAL FLAGS
bool _HAS_MAP   = false;    // 是否有地图
bool _HAS_ODOM  = false;    // 是否有里程计信息
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
Vector3d _first_pose;

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
void rcvPointCloudCallback(const sensor_msgs::PointCloud2 & pointcloud_map);
// 里程计回调函数，负责设置机器人位置;进行轨迹跟踪
void rcvOdomCallback(const nav_msgs::Odometry::ConstPtr& msg);

// 关键功能函数 ############################################################################################################
// search trajectory checking
bool TrackTrajCheck();
// 搜索到的 tracksegs 从 lazykinoprm 类 push 到 searchTraj 列表
void SearchSegPush();
// 优化后的 tracksegs 从 optimalTraj 列表 pop 到 tracking 类中
void OptimalSegPop();
// 待优化的 tracksegs 从 searchTraj 列表 push 到 optimalTraj 列表
void OptimalSegPush();
// 轨迹跟踪的安全性检查
bool NavSeqCheck(const nav_msgs::Path *navseq);
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
    _map_sub  = nh.subscribe( "map",       1, rcvPointCloudCallback );
    _pts_sub  = nh.subscribe( "waypoints", 1, rcvWaypointsCallback );
    _odom_sub = nh.subscribe( "odom",      1, rcvOdomCallback );

    // 路径和地图可视化
    _path_vis_pub             = nh.advertise<visualization_msgs::MarkerArray>("planner_path_vis",1);
    // _obsmap_img_pub           = nh.advertise<sensor_msgs::Image>("obs_map_img",1);
    _sdfmap_img_pub           = nh.advertise<sensor_msgs::Image>("sdf_map_img",1);
    _robot_vis_pub            = nh.advertise<visualization_msgs::MarkerArray>("robot_vis",1);
    // _cmd_vel_pub              = nh.advertise<geometry_msgs::Twist>("/cmd_vel",1);
    // _nav_seq_vis_pub          = nh.advertise<nav_msgs::Path>("/nav_seq_vis",1);

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
    nh.param("map/resolution", _resolution, 0.05); // obs_mao grid 5cm

    nh.param("search/time_interval",_time_interval,0.01);
    // 轨迹跟踪相关参数
    nh.param("planner/loop_rate", _loop_rate, 1.0);
    nh.param("planner/vis_Robot", _vis_Robot, false);
    nh.param("planner/vis_sample", _vis_sample, false);
    nh.param("planner/vis_traj", _vis_traj, false);
    nh.param("planner/nav_seq_vis", _nav_seq_vis_flag, true);
    // nh.param("planner/time_interval", _time_interval, 0.1); 
    nh.param("planner/optHorizon", _optHorizon, 1.5); 
    // 直接根据 loop_rate 计算每次跟踪的时间间隔
    // _time_interval = double( 1 / _loop_rate);
    if (_DIST_RES > _resolution)
        _DIST_RES = _resolution * 0.8; 
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
    ROS_INFO("[\033[34mPlanNode\033[0m]_local_width: %2.4f",    _local_width);
    ROS_INFO("[\033[34mPlanNode\033[0m]_resolution : %2.4f",    _resolution);
    ROS_INFO("[\033[34mPlanNode\033[0m]_loop_rate  : %2.4f",    _loop_rate);
    ROS_INFO("[\033[34mPlanNode\033[0m]_time_interval: %2.4f",  _time_interval);
    ROS_INFO("[\033[34mPlanNode\033[0m]_optHorizon  : %2.4f",   _optHorizon);
    ROS_INFO("[\033[34mPlanNode\033[0m]_opt_seg     : %6d",     _OPT_SEG);
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
        _map_sub_cnt++;
        if (_map_sub_cnt > SUB_CNT_TH) {
            _HAS_MAP = false;
            ROS_INFO("[\033[31mPlanNode\033[0m]: !!! don't receive map !!!");
            _map_sub_cnt = 5;
        }
        _odom_sub_cnt++;
        if (_odom_sub_cnt > SUB_CNT_TH) {
            _HAS_ODOM = false;
            ROS_INFO("[\033[31mPlanNode\033[0m]: !!! don't receive odom !!!");
            _odom_sub_cnt = 5;
        }
        // step 轨迹的安全性检查 ############################################################################################################
        // 检查接下来需要跟踪的轨迹是否安全
        if (!_REACH_GOAL){
            bool feasible = TrackTrajCheck();
            // step.1 判断是否距离障碍物太近 就得急刹车了
            if (tracking.TROT_FLAG){ // 障碍物距离太近,需要急刹车再重新规划
                _TRACKING = false;
                tracking.NavSeqFixed(tracking._local_odom);
                tracking.NavSeqPublish();
                ROS_INFO("[\033[31mPlanNode\033[0m]: too close to obstacle, braking!!!");
            }
            // step.2 轨迹上有障碍物,正常重规划
            if (!feasible){  // 轨迹上有障碍物
                ROS_INFO("[\033[33mPlanNode\033[0m]: track trajectory is not feasible");
                _HAS_PATH = false;
                Vector3d goalPose,currPose,currVel,currAcc,zeros_pt(0,0,0);
                if(tracking.getReplanState(&currPose,&currVel,&currAcc,&goalPose)){
                    _HAS_PATH = !lazykinoPRM.search(currPose,currVel,currAcc,goalPose,zeros_pt,zeros_pt);
                }
                else {
                    ROS_INFO("[\033[31mPlanNode\033[0m]: get replan state failed");
                    continue;
                }
                if (_HAS_PATH){ // 重新规划成功
                    tracking._goalPose = lazykinoPRM._goal_pose;
                    SearchSegPush();
                    tracking.insertSegTraj(tracking._search_seg_index,&searchTraj);
                    ROS_INFO("[\033[32mPlanNode\033[0m]: replan success");
                    tracking.OBS_FLAG = false;
                    _TRACKING = true;
                    visSearchTraj(); // 可视化搜索轨迹 感觉就是DEBUG的时候用
                    // 可视化轨迹后再清空搜索列表
                    // ROS_INFO("[\033[34mPlanNode\033[0m]: reset planning done");
                    lazykinoPRM.reset();
                }
                else{ // 重新规划失败
                    // TODO: 重新规划失败解决方案 1.重采样再规划 2.刹车等待摆烂
                    ROS_INFO("[\033[31mPlanNode\033[0m]: replan failed");
                }
            }
        }
        // step 轨迹的优化 ##########################################################################################################
        if (!_REACH_GOAL){

        }

        // step 可视化 ############################################################################################################
        // 可视化机器人和感知范围
        if (_vis_Robot){
            visRobot(currodometry);
            // ROS_INFO("[\033[32mPlanNode\033[0m]: vis robot");
            _vis_Robot=false;
        }
        if (_vis_traj){
            visPtraj();
            visVtraj();
            _vis_traj=false;
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
void rcvWaypointsCallback(const nav_msgs::Path &wp)
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

    if (_HAS_PATH) {
        _NEW_PATH = true;
        tracking.initPlanner(goalPose);
        tracking._goalPose = lazykinoPRM._goal_pose;
        SearchSegPush();
        tracking.insertSegTraj(0,&searchTraj);
        ROS_INFO("[\033[32mPlanNode\033[0m]: insertSegTraj success");
        // save trajectory as .CSV
        // csvPtraj();
    }
    // ####################################################################################
    visSearchTraj();
    // 可视化轨迹后再清空搜索列表
    lazykinoPRM.reset();
    ROS_INFO("[\033[34mPlanNode\033[0m]: reset planning done");
}

// 点云回调函数，负责设置障碍物
void rcvPointCloudCallback(const sensor_msgs::PointCloud2 & pointcloud_map)
{   
    _map_sub_cnt--;
    if (_map_sub_cnt < 0)
        _map_sub_cnt = 5;

    pcl::PointCloud<pcl::PointXYZ> cloud;
    sensor_msgs::PointCloud2 map_vis;

    pcl::fromROSMsg(pointcloud_map, cloud);

    // ROS_DEBUG("[Node]cloud.points.size() = %d", (int)cloud.points.size());
    
    if( static_cast<int>(cloud.points.size()) == 0 ) { // 如果点云为空，就不进行地图更新，但是可能是正常的(没有障碍物)
        ROS_INFO("[\033[33m !!!!! rcvPCL\033 none points !!!!![0m]:");
        return;
    }
    lazykinoPRM.updataObsMap(cloud);
    cv::Mat* obsimg = lazykinoPRM.getObsMap();
    // cv::Mat obsptr = *lazykinoPRM.obs_map;
    
    // ROS_DEBUG("[Node]obsimg.size() = %d, %d", obsimg->rows, obsimg->cols);
    
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
void rcvOdomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    //// 必要的处理 标志位的设置
    _odom_sub_cnt--;
    if (_odom_sub_cnt < 0)
        _odom_sub_cnt = 5;

    currodometry = msg;
    _vis_Robot = true;
    _HAS_ODOM  = true;
    tracking._currodometry = msg;

    // 轨迹控制 跟踪位置/速度
    Vector3d _current_odom;
    _current_odom(0) = msg->pose.pose.position.x;
    _current_odom(1) = msg->pose.pose.position.y;
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
        ROS_INFO("[\033[32mPlannerNode\033[0m]: first odom: %2.2f, %2.2f, %2.2f", _first_pose(0), _first_pose(1), _first_pose(2));
        _FIRST_ODOM = false;
    }
    // _current_odom(2) 的范围是 -pi ~ pi
    // ####################################################################################
    if (_HAS_PATH && !_REACH_GOAL){ // 如果有路径,但是没到终点,就进行轨迹跟踪
        tracking._tracking_cnt++;
        if (tracking.isReachGoal(_current_odom)){
            _REACH_GOAL = true;
            ROS_INFO("[\033[32mPlannerNode\033[0m]: reach goal");
        }
        else {
            _REACH_GOAL = false;
            // 计算当前odometry在轨迹上的位置
            tracking.OdometryIndex(_current_odom);
            // 更新轨迹跟踪的控制序列
            tracking.NavSeqUpdate();
            if (!NavSeqCheck(&tracking._nav_seq_msg)){
                ROS_INFO("[\033[33mPlannerNode\033[0m]: NavSeqCheck failed!");
                tracking.NavSeqFixed(tracking._local_odom);
            }
        }
    }
    tracking._current_odom = _current_odom;
    if (_REACH_GOAL){
        tracking.NavSeqFixed(tracking._goalPose);
    }
    else{
        // 如果没有路径,就不进行轨迹跟踪
        tracking.NavSeqFixed(tracking._local_odom);
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
bool TrackTrajCheck() // TODO: add BAIS_FLAG for trajectory check NMPC 跟踪效果很好感觉没啥必要
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
void SearchSegPush()
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

/*******************************************************************************************
 * @description: 优化后的 tracksegs 从 optimalTraj 列表 pop 到 tracking 类中
 * @reference: 
 * @return {*}
 */
void OptimalSegPop()
{

}

//########################################################################################
// 可视化函数


void visPtraj()
{
    nav_msgs::Path path;
    path.header.frame_id = "world";
    path.header.stamp = ros::Time::now();
}

void visVtraj()
{

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
 * @description:  可视化搜索到的: 轨迹 采样点 终点
 * @reference: 
 * @return {*}
 */
void visSearchTraj()
{
    double _vis_resolution = 0.02;
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
    Line.scale.x         = _vis_resolution;

    Line.color.r         = 0.0;
    Line.color.g         = 0.0;
    Line.color.b         = 1.0;
    Line.color.a         = 1.0;

    Spheres.pose.orientation.w = 1.0;
    Spheres.type            = visualization_msgs::Marker::SPHERE_LIST;
    Spheres.scale.x         = _vis_resolution*5;
    Spheres.scale.y         = _vis_resolution*5;
    Spheres.scale.z         = _vis_resolution*5;

    Spheres.color.a         = 0.8;
    Spheres.color.r         = 0.0;
    Spheres.color.g         = 0.0;
    Spheres.color.b         = 1.0;

    ESpheres.pose.orientation.w = 1.0;
    ESpheres.type            = visualization_msgs::Marker::SPHERE_LIST;
    ESpheres.scale.x         = _vis_resolution*5;
    ESpheres.scale.y         = _vis_resolution*5;
    ESpheres.scale.z         = _vis_resolution*5;

    ESpheres.color.a         = 0.6;
    ESpheres.color.r         = 1.0;
    ESpheres.color.g         = 0.0;
    ESpheres.color.b         = 0.0;

    OSpheres.pose.orientation.w = 1.0;
    OSpheres.type            = visualization_msgs::Marker::SPHERE_LIST;
    OSpheres.scale.x         = _vis_resolution*5;
    OSpheres.scale.y         = _vis_resolution*5;
    OSpheres.scale.z         = _vis_resolution*5;

    OSpheres.color.a         = 0.6;
    OSpheres.color.r         = 0.0;
    OSpheres.color.g         = 1.0;
    OSpheres.color.b         = 0.0;
    
    GSphere.pose.orientation.w = 1.0;
    GSphere.type            = visualization_msgs::Marker::SPHERE;
    GSphere.scale.x         = _vis_resolution*2;
    GSphere.scale.y         = _vis_resolution*2;
    GSphere.scale.z         = _vis_resolution*2;

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
            for (int i = 0; i < static_cast<int>(xtraj.size()); i++)
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

        for (int idx = 0; idx < static_cast<int>(lazykinoPRM.pathstateSets.size()); idx++)
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
        for (int idx = 0; idx < static_cast<int>(lazykinoPRM.astaropenlist.nodestateSets.size()); idx++)
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
        ROS_DEBUG("[visSearchTraj]LineArray.size() = %d", static_cast<int>(LineArray.markers.size()));
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
    double _vis_resolution = 0.02;
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
    Line.scale.x         = _vis_resolution;

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

    LineArray.markers.push_back(Spheres);

    _robot_vis_pub.publish(LineArray);
}

/********************************************************************************/