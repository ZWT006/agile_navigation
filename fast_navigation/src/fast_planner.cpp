/*
 * @Author: wentao zhang && zwt190315@163.com
 * @Date: 2023-04-03
 * @LastEditTime: 2023-06-15
 * @Description: 
 * @reference: 
 * 
 * Copyright (c) 2023 by wentao zhang, All Rights Reserved. 
 */

/*
 *TODO LIST:
 * 1. add a new class to handle the path tracking (确实有必要，因为当前代码的功能太多了很乱)
 * 2. add a functiom to assess the tracking performance (计算跟踪误差的方差,均值等[px,py,pq],[vx,vy,vq])
 * 3. add a function to handle the path replanning (包括路径的safe check,路径的replan,路径的replan后的跟踪)
 * 4. add Trajcetory visualization (路径的可视化,包括路径的可视化,跟踪的可视化,跟踪误差的可视化,跟踪误差的统计量的可视化)?
 * 5. add Trajectory Optimization (可以考虑使用osqp,qpOASES,qpOASES-3.2.1,qpOASES-3.2.1,qpOASES-3.2.1)也许新建一个功能包比较好
 * 6. pref osbmap and esdf ,it's map parts(优化osbmap部分的代码)
*/
#include <fast_navigation/navigation.hpp>
#include <lazykinoprm/LazyKinoPRM.h>

using namespace std;
using namespace Eigen;
using namespace cv;

// visualization Navigation Sequences
#define VIS_NAV_SEQ
#define DEFAULT_HIGH 0.3f

#define Q_WEIGHT 0.01f

// visualization_msgs::Marker 可视化的透明度
#define RGB_ALPHA 0.6


// ros related
ros::Subscriber _map_sub, _pts_sub, _odom_sub;
ros::Publisher _cmd_vel_pub,_nav_seq_pub,_nav_seq_vis_pub;
ros::Publisher _obsmap_img_pub,_sdfmap_img_pub;
ros::Publisher _ptraj_vis_pub,_vtraj_vis_pub;
ros::Publisher  _grid_map_vis_pub, _path_vis_pub,_robot_vis_pub;

// node parameters
double _local_w,_sense_rate;
double robot_width=0.30,robot_length=0.40; //unitree A1 robot size
double robot_height=0.15;

// simulation param from launch file
double _resolution, _inv_resolution, _cloud_margin;
double _x_size, _y_size, _z_size;
double _x_orign, _y_orign, _z_orign;
Vector3d _start_pt, _start_velocity;

// useful global variables
bool _has_map   = false;
bool _has_path  = false;
bool _new_path = true;
bool _first_odom = true;
vector<double> xtraj,ytraj,qtraj;
vector<double> pxtraj,pytraj,pqtraj;
vector<double> vxtraj,vytraj,vqtraj;
int _odom_cnt = 0;
int _odom_cnt_th = 10;
int _tracking_cnt = 0;

nav_msgs::Odometry::ConstPtr currodom;
// visualization robot switch 在ros循环中可视化 
bool _visRobot=false;
bool _nav_seq_vis_flag=true;
// 跟踪轨迹的pos可视化
nav_msgs::Path _nav_seq_vis_msg;

// Integral parameter
double _max_input_acc     = 1.0;
int    _discretize_step   = 2;
double _time_interval     = 0.01;


// planner parameters
double _time_horizon = 1.0;
double _traj_time_interval = 0.1;
// int _traj_numth = 0;
// double traj_referT = 0.0;

int _time_step_interval = 10; // 
int _time_step_pursuit = 5;
int _curr_time_step   = 0;
int _goal_time_step   = 0;
double _vel_factor = 1.0;
// geometry_msgs::Twist _cmd_vel;
// odom 的频率为10hz _time_interval=0.01s so _time_step_interval=10

// close loop for path following
double _Kp_x, _Kp_y, _Kp_q;
//最简单的Kp参数, (s_ref - s_get)/_traj_time_interval = Kp_s * (s_ref - s_get)
// Kp_s = 1/_traj_time_interval = 10
double _max_vel_x,_max_vel_y,_max_vel_q;
double _refer_vel,_refer_ome;

Vector3d _currPose,_localPose,_goalPose;



// Lazy Kinodynamic Path Searching
LazyKinoPRM lazykinoPRM;

// 目标点回调函数，进行路径规划
void rcvWaypointsCallback(const nav_msgs::Path & wp);
// 点云回调函数，负责设置障碍地图并更新SDF地图
void rcvPointCloudCallBack(const sensor_msgs::PointCloud2 & pointcloud_map);
// 里程计回调函数，负责设置机器人位置;进行轨迹跟踪
void rcvOdomCallBack(const nav_msgs::Odometry::ConstPtr& msg);

// 轨迹跟踪 根据当前odometry计算期望 cmd_vel
void cmdTracking(const nav_msgs::Odometry::ConstPtr& msg);
// 轨迹跟踪 根据当前odometry计算期望 trajectory
void navTracking(const nav_msgs::Odometry::ConstPtr& msg);

// 可视化函数
void visTraLibrary();
// visulization robot 
void visRobotNode(const nav_msgs::Odometry::ConstPtr& msg);
// visulization position
void visPtraj();
// visulization velocity 
void visVtraj();
// save trajectory as .csv file
void csvPtraj();

int main(int argc, char** argv)
{
    ros::init(argc, argv, "planner_node");
    ros::NodeHandle nh("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);

    // 地图和导航点订阅
    _map_sub  = nh.subscribe( "map",       1, rcvPointCloudCallBack );
    _pts_sub  = nh.subscribe( "waypoints", 1, rcvWaypointsCallback );
    _odom_sub = nh.subscribe( "odom",      1, rcvOdomCallBack );

    // 路径和地图可视化
    // _grid_map_vis_pub         = nh.advertise<sensor_msgs::PointCloud2>("grid_map_vis", 1);
    _path_vis_pub             = nh.advertise<visualization_msgs::MarkerArray>("planner_path_vis",1);
    // _obsmap_img_pub           = nh.advertise<sensor_msgs::Image>("obs_map_img",1);
    _sdfmap_img_pub           = nh.advertise<sensor_msgs::Image>("sdf_map_img",1);
    _robot_vis_pub            = nh.advertise<visualization_msgs::MarkerArray>("robot_vis",1);
    _cmd_vel_pub              = nh.advertise<geometry_msgs::Twist>("/cmd_vel",1);
    _nav_seq_pub              = nh.advertise<nav_msgs::Path>("/nav_seq",1);
    _nav_seq_vis_pub          = nh.advertise<nav_msgs::Path>("/nav_seq_vis",1);

    _ptraj_vis_pub            = nh.advertise<nav_msgs::Path>("/ptraj_vis",1);
    _vtraj_vis_pub            = nh.advertise<visualization_msgs::MarkerArray>("/vtraj_vis",1);

    ROS_DEBUG("lazykinoPRM.setParam(nh)");
    // LazyKinoPRM 启动参数
    lazykinoPRM.setParam(nh);
    // 初始化相关参数
    ROS_DEBUG("lazykinoPRM.init()");
    lazykinoPRM.init();
    // 地图相关参数
    nh.param("map/local_w", _local_w, 2.0);
    nh.param("sensing/rate", _sense_rate, 1.0);
    nh.param("planner/nav_seq_vis", _nav_seq_vis_flag, true);


    nh.param("planner/time_horizon", _time_horizon, 1.0);
    nh.param("planner/time_step_interval", _time_step_interval, int(10.0));
    nh.param("planner/time_step_pursuit", _time_step_pursuit, int(5.0));
    nh.param("planner/refer_vel",_refer_vel,2.0);
    nh.param("planner/refer_ome", _refer_ome,1.57);

    nh.param("planner/Kp_x", _Kp_x, 1.0);
    nh.param("planner/Kp_y", _Kp_y, 1.0);
    nh.param("planner/Kp_q", _Kp_q, 1.0);
    nh.param("planner/max_vel_x", _max_vel_x, 2.0);
    nh.param("planner/max_vel_y", _max_vel_y, 1.0);
    nh.param("planner/max_vel_q", _max_vel_q, 1.57);

    _vel_factor = double(_time_step_pursuit) / double(_time_step_interval);

    ROS_INFO("[\033[34mPlanNode\033[0m]planner/time_horizon: %2.4f", _time_horizon);
    ROS_INFO("[\033[34mPlanNode\033[0m]planner/traj_time_interval: %2.4f", _traj_time_interval);
    ROS_INFO("[\033[34mPlanNode\033[0m]planner/time_step_interval: %d", _time_step_interval);
    ROS_INFO("[\033[34mPlanNode\033[0m]planner/time_step_pursuit: %d", _time_step_pursuit);
    ROS_INFO("[\033[34mPlanNode\033[0m]planner/vel_factor: %2.4f", _vel_factor);

    ros::Rate rate(_sense_rate);
    bool status = ros::ok();
    while(status) 
    {
        ros::spinOnce();
        if (_visRobot)
        {
            visRobotNode(currodom);
            _visRobot=false;
        }
            // 机器人控制指令
        if (_nav_seq_vis_flag)
        {
            _nav_seq_vis_pub.publish(_nav_seq_vis_msg);
            // 发送后就及时清空
            _nav_seq_vis_msg.poses.clear();
            // ROS_INFO("[\033[34mPlannerNode\033[0m]_nav_seq_msg size: %d", _nav_seq_msg.poses.size());
            // ROS_INFO("[\033[34mPlannerNode\033[0m]_nav_seq : %d", _nav_seq);
        }
        status = ros::ok();
        rate.sleep();
    }

    return 0;
}

// 导航点回调函数，进行路径规划
void rcvWaypointsCallback(const nav_msgs::Path & wp)
{     
    if( _has_map == false )
        return;
    double yaw_rad,yaw_deg;
    Vector3d target_pt,current_pt;
    // target_pt << wp.poses[0].pose.position.x,
    //              wp.poses[0].pose.position.y,
    //              wp.poses[0].pose.position.z;
    target_pt(0) = wp.poses[0].pose.position.x;
    target_pt(1) = wp.poses[0].pose.position.y;
    target_pt(2) = tf::getYaw(wp.poses[0].pose.orientation); // use tf2 library to get yaw from quaternion
    
    Vector3d zeros_pt(0,0,0);
    ROS_INFO("[\033[34mPlanNode\033[0m] receive the planning target");
    ROS_INFO("[\033[34mTarget\033[0m]: %f, %f, %f", wp.poses[0].pose.position.x, wp.poses[0].pose.position.y, wp.poses[0].pose.position.z);
    ROS_INFO("[\033[34mTarget\033[0m]:orientation %f, %f, %f, %f", wp.poses[0].pose.orientation.x, wp.poses[0].pose.orientation.y, wp.poses[0].pose.orientation.z, wp.poses[0].pose.orientation.w);
    ROS_INFO("[\033[34mTarget\033[0m]: %f, %f, %f", target_pt(0), target_pt(1), target_pt(2));
    yaw_rad = target_pt(2);
    yaw_deg = yaw_rad * 180 / M_PI;
    _goalPose = target_pt;
    current_pt = _currPose;

    ROS_INFO("[\033[34mTarget\033[0m]:yaw angle %f", yaw_deg);
    _has_path = lazykinoPRM.search(current_pt,zeros_pt,zeros_pt,target_pt,zeros_pt,zeros_pt);
    _has_path = !_has_path;
    if (_has_path)
    {
        _new_path = true;
        double trajectory_length = 0, trajectory_time = 0;
        pxtraj.clear();pytraj.clear();pqtraj.clear();
        vxtraj.clear();vytraj.clear();vqtraj.clear();
        for (int idx = 0; idx < int(lazykinoPRM.pathstateSets.size()); idx++)
        {
            vector<double> _xtraj, _ytraj, _qtraj;
            vector<double> _dxtraj, _dytraj, _dqtraj;
            _xtraj = lazykinoPRM.pathstateSets[idx].polytraj('p', 'x', _time_interval);
            _ytraj = lazykinoPRM.pathstateSets[idx].polytraj('p', 'y', _time_interval);
            _qtraj = lazykinoPRM.pathstateSets[idx].polytraj('p', 'q', _time_interval);
            _dxtraj = lazykinoPRM.pathstateSets[idx].polytraj('v', 'x', _time_interval);
            _dytraj = lazykinoPRM.pathstateSets[idx].polytraj('v', 'y', _time_interval);
            _dqtraj = lazykinoPRM.pathstateSets[idx].polytraj('v', 'q', _time_interval);
            pxtraj.insert(pxtraj.end(), _xtraj.begin(), _xtraj.end());
            pytraj.insert(pytraj.end(), _ytraj.begin(), _ytraj.end());
            pqtraj.insert(pqtraj.end(), _qtraj.begin(), _qtraj.end());
            vxtraj.insert(vxtraj.end(), _dxtraj.begin(), _dxtraj.end());
            vytraj.insert(vytraj.end(), _dytraj.begin(), _dytraj.end());
            vqtraj.insert(vqtraj.end(), _dqtraj.begin(), _dqtraj.end());
            trajectory_length += lazykinoPRM.pathstateSets[idx].trajectory_length;
            trajectory_time += lazykinoPRM.pathstateSets[idx].referT;
        }
        ROS_INFO("[\033[32mPlanNode\033[0m]: trajectory length: %2.4f", trajectory_length);
        ROS_INFO("[\033[32mPlanNode\033[0m]: trajectory time: %2.4f", trajectory_time);
        // save trajectory as .CSV
        csvPtraj();
    }
    visTraLibrary();
    // 可视化轨迹后再清空搜索列表
    ROS_INFO("[\033[34mPlanNode\033[0m]: reset planning done");
    lazykinoPRM.reset();
    _has_map = false;
}

// 点云回调函数，负责设置障碍物
void rcvPointCloudCallBack(const sensor_msgs::PointCloud2 & pointcloud_map)
{   
    // 很明显这个函数就运行一次
    if(_has_map ) return;

    pcl::PointCloud<pcl::PointXYZ> cloud;
    sensor_msgs::PointCloud2 map_vis;

    pcl::fromROSMsg(pointcloud_map, cloud);

    ROS_DEBUG("[Node]cloud.points.size() = %d", (int)cloud.points.size());
    
    if( (int)cloud.points.size() == 0 ) return;

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
    _has_map = true;
}

// 里程计回调函数，负责设置机器人位置;进行轨迹跟踪
void rcvOdomCallBack(const nav_msgs::Odometry::ConstPtr& msg)
{
    // 发布机器人位姿
    currodom = msg;
    _visRobot = true;

    // 轨迹控制 跟踪位置
    navTracking(msg);
    // 速度控制 跟踪位置
    //cmdTracking(msg);
}



// 这里要完整的修改,轨迹点的位置与速度信息缩放必须好好考虑清楚
void navTracking(const nav_msgs::Odometry::ConstPtr &msg)
{
    // if( _has_path == false ) return;
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

    _nav_seq_vis_msg.header.frame_id = "world";
    _nav_seq_vis_msg.header.stamp = ros::Time::now();
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
    if (_first_odom)
    {
        _localPose = _currPose;
        ROS_DEBUG("[PlannerNode]:_localPose = %f, %f, %f", _localPose(0), _localPose(1), _localPose(2));
        _first_odom = false;
    }

    if (_has_path)
    {
        // 如果已经到达目标点，就不再进行轨迹跟踪 
        if (nearPose(Vector2d(_x_get, _y_get), Vector2d(_goalPose(0), _goalPose(1)), _q_get, _goalPose(2)))
        {
            ROS_INFO("[\033[32mPlannerNode\033[0m]: goal reached!");
            ROS_INFO("[\033[32mPlannerNode\033[0m]: tacking time = %2.4f", double(_tracking_cnt)*_traj_time_interval);
            _has_path = false;
            _new_path = true;
            _localPose = _goalPose;
            _tracking_cnt = 0;
            return;
        }
        _tracking_cnt ++;
        // 如果跟踪的是新的轨迹，就重新设置轨迹
        if (_new_path)
        {
            _new_path = false;
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
        }     // 这里的else是if (_new_path)的else
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
    else // 这个else是if (_has_path)的else
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

void cmdTracking(const nav_msgs::Odometry::ConstPtr &msg)
{
    // if( _has_path == false ) return;
    geometry_msgs::Twist cmd_vel;
    double _x_get, _y_get, _q_get;
    double _x_ref, _y_ref, _q_ref;
    double x_vel, y_vel, q_vel;
    // double _x_err,_y_err,_q_err;
    _x_get = msg->pose.pose.position.x;
    _y_get = msg->pose.pose.position.y;
    _q_get = tf::getYaw(msg->pose.pose.orientation); // use tf2 library to get yaw from quaternion
    // _q_get 的范围是 -pi ~ pi

    _currPose << _x_get, _y_get, _q_get;

    // 初始的_odom_cnt_th 个里程计信息不使用
    if (_odom_cnt < _odom_cnt_th)
    {
        _odom_cnt++;
        return;
    }

    // 如果是第一次接收到里程计信息，就将当前位置设置为起始位置
    if (_first_odom)
    {
        _localPose = _currPose;
        ROS_DEBUG("[Node]:_localPose = %f, %f, %f", _localPose(0), _localPose(1), _localPose(2));
        _first_odom = false;
    }

    if (_has_path)
    {
        // 如果跟踪的是新的轨迹，就重新设置轨迹
        if (_new_path)
        {
            _new_path = false;
            _curr_time_step = 0;
            _goal_time_step = _time_step_pursuit;
            _x_ref = pxtraj.at(_curr_time_step);
            _y_ref = pytraj.at(_curr_time_step);
            _q_ref = pqtraj.at(_curr_time_step);
        }
        else
        {
            // 当前轨迹跟踪的时间步还在轨迹内
            if (_goal_time_step + _time_step_pursuit < pqtraj.size())
            {
                vector<double> dists;
                // 在_time_step_interval步长范围内，搜索当前/odometry最近的轨迹点
                for (int idx = _goal_time_step - _time_step_interval / 2; idx <= _goal_time_step + _time_step_interval / 2; idx++)
                {
                    if (idx < 0 || idx >= pqtraj.size())
                    {
                        ROS_INFO("[\033[33mPlannerNode\033[0m]: idx = %d", idx);
                        continue;
                    }
                    double dist = sqrt(pow(_x_get - pxtraj.at(idx), 2) + pow(_y_get - pytraj.at(idx), 2));
                    dists.push_back(dist);
                }
                // 找到最近的轨迹点
                int min_idx = min_element(dists.begin(), dists.end()) - dists.begin();
                _curr_time_step = _goal_time_step - _time_step_interval / 2 + min_idx;
                // TODO: 这里存在Bug,如果最近的轨迹点在前面，就会导致跟踪的轨迹点后退
                // 还有一种情况是,_curr_time_step 的点距离odometry太远,这时候也需要重新设置轨迹
                // 设置当前轨迹跟踪的时间步 = 当前时间步 + 跟踪步长
                _goal_time_step = _curr_time_step + _time_step_pursuit;
                if (_goal_time_step >= pqtraj.size())
                    _goal_time_step = pqtraj.size() - 1;
                _x_ref = pxtraj.at(_goal_time_step);
                _y_ref = pytraj.at(_goal_time_step);
                _q_ref = pqtraj.at(_goal_time_step);
            }
            else // 当前轨迹跟踪的时间步已经超出轨迹范围,就跟踪最后一个轨迹点
            {
                _goal_time_step = pqtraj.size();
                _x_ref = pxtraj.back();
                _y_ref = pytraj.back();
                _q_ref = pqtraj.back();
                _has_path = false;
                ROS_INFO("[\033[34mOdomCallBack\033[0m]: path tracking done,near the goal");
            }
        }
        _localPose(0) = _x_ref;
        _localPose(1) = _y_ref;
        _localPose(2) = _q_ref;
    }
    else
    {
        _x_ref = _localPose(0);
        _y_ref = _localPose(1);
        _q_ref = _localPose(2);
    }
    // 如果当前位置已经在目标位置附近，就停止
    if (nearPose(Vector2d(_x_get, _y_get), Vector2d(_x_ref, _y_ref), _q_get, _q_ref))
    {
        cmd_vel.linear.x = 0.0;
        cmd_vel.linear.y = 0.0;
        cmd_vel.angular.z = 0.0;
    }
    else
    {
        // 计算速度
        x_vel = (_x_ref - _x_get) * _Kp_x;
        y_vel = (_y_ref - _y_get) * _Kp_y;
        // q_vel = (_q_ref - _q_get) * _Kp_q;
        q_vel = angles::shortest_angular_distance(_q_get, _q_ref) * _Kp_q;

        x_vel = cos(_q_get) * x_vel + sin(_q_get) * y_vel; // 转换到机器人坐标系下
        y_vel = -sin(_q_get) * x_vel + cos(_q_get) * y_vel;

        // 限制速度
        if (x_vel > _max_vel_x)
            x_vel = _max_vel_x;
        else if (x_vel < -_max_vel_x)
            x_vel = -_max_vel_x;
        if (y_vel > _max_vel_y)
            y_vel = _max_vel_y;
        else if (y_vel < -_max_vel_y)
            y_vel = -_max_vel_y;
        if (q_vel > _max_vel_q)
            q_vel = _max_vel_q;
        else if (q_vel < -_max_vel_q)
            q_vel = -_max_vel_q;
        // 如果最大速度超过了限制，则将三个速度按比例缩小
        // double max_vel;
        // max_vel = abs(x_vel);
        // if (max_vel > _max_vel_x) {
        // double scale_factor = _max_vel_x / max_vel;
        // x_vel *= scale_factor;
        // y_vel *= scale_factor;
        // q_vel *= scale_factor;
        // }
        // max_vel = abs(y_vel);
        // if (max_vel > _max_vel_y) {
        // double scale_factor = _max_vel_y / max_vel;
        // x_vel *= scale_factor;
        // y_vel *= scale_factor;
        // q_vel *= scale_factor;
        // }
        // max_vel = abs(q_vel);
        // if (max_vel > _max_vel_q) {
        // double scale_factor = _max_vel_q / max_vel;
        // x_vel *= scale_factor;
        // y_vel *= scale_factor;
        // q_vel *= scale_factor;
        // }

        // 发布速度
        cmd_vel.linear.x = x_vel;
        cmd_vel.linear.y = y_vel;
        cmd_vel.angular.z = q_vel;
    }
    // 机器人控制指令
    _cmd_vel_pub.publish(cmd_vel);
}

//#####################################################################################################
// 可视化

void visPtraj()
{
    nav_msgs::Path path;
    path.header.frame_id = "world";
    path.header.stamp = ros::Time::now();
    path.poses.resize(pqtraj.size()/5);
    for (int i = 0; i < pqtraj.size(); i=i+5)
    {
        path.poses[i].pose.position.x = pxtraj.at(i);
        path.poses[i].pose.position.y = pytraj.at(i);
        path.poses[i].pose.position.z = DEFAULT_HIGH;
        path.poses[i].pose.orientation = tf::createQuaternionMsgFromYaw(pqtraj.at(i));
    }
    _ptraj_vis_pub.publish(path);
}

void visVtraj()
{

}

void csvPtraj()
{
    ofstream outfile;
    outfile.open("/home/zwt/catkin_ws/src/navigation/fast_navigation/datas/ptraj.csv");
    if (outfile)
    {
        outfile.close();
        outfile.open("/home/zwt/catkin_ws/src/navigation/fast_navigation/datas/ptraj.csv", ios_base::out | ios_base::trunc);
    }
    outfile << "px,py,pq,vx,vy,vq" << endl;
    ROS_INFO("[\033[35mNodeCSV\033[0m] datas.size() = %d", pxtraj.size());
    for (int i = 0; i < pqtraj.size(); i++)
    {
        outfile << pxtraj.at(i) << "," << pytraj.at(i) << "," << pqtraj.at(i) << ","
             << vxtraj.at(i) << "," << vytraj.at(i) << "," << vqtraj.at(i) << endl;
    }
    outfile.close();
}



void visTraLibrary()
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

            // #######################################################################################
            //  double xi_pos,yi_pos,qi_pos,xf_pos,yf_pos,qf_pos;
            //  double xi_vel,yi_vel,qi_vel,xf_vel,yf_vel,qf_vel;
            //  double xi_acc,yi_acc,qi_acc,xf_acc,yf_acc,qf_acc;
            //  Vector3d svi,sai,svf,saf,spi,spf;
            //  svi = lazykinoPRM.pathstateSets[idx].PareVelocity;
            //  sai = lazykinoPRM.pathstateSets[idx].PareAcceleration;
            //  svf = lazykinoPRM.pathstateSets[idx].Velocity;
            //  saf = lazykinoPRM.pathstateSets[idx].Acceleration;
            //  spi = lazykinoPRM.pathstateSets[idx].ParePosition;
            //  spf = lazykinoPRM.pathstateSets[idx].Position;

            // xi_pos = lazykinoPRM.pathstateSets[idx].polytrajStart('p','x');
            // yi_pos = lazykinoPRM.pathstateSets[idx].polytrajStart('p','y');
            // qi_pos = lazykinoPRM.pathstateSets[idx].polytrajStart('p','q');
            // xf_pos = lazykinoPRM.pathstateSets[idx].polytrajEnd('p','x');
            // yf_pos = lazykinoPRM.pathstateSets[idx].polytrajEnd('p','y');
            // qf_pos = lazykinoPRM.pathstateSets[idx].polytrajEnd('p','q');
            // xi_vel = lazykinoPRM.pathstateSets[idx].polytrajStart('v','x');
            // yi_vel = lazykinoPRM.pathstateSets[idx].polytrajStart('v','y');
            // qi_vel = lazykinoPRM.pathstateSets[idx].polytrajStart('v','q');
            // xi_acc = lazykinoPRM.pathstateSets[idx].polytrajStart('a','x');
            // yi_acc = lazykinoPRM.pathstateSets[idx].polytrajStart('a','y');
            // qi_acc = lazykinoPRM.pathstateSets[idx].polytrajStart('a','q');
            // xf_vel = lazykinoPRM.pathstateSets[idx].polytrajEnd('v','x');
            // yf_vel = lazykinoPRM.pathstateSets[idx].polytrajEnd('v','y');
            // qf_vel = lazykinoPRM.pathstateSets[idx].polytrajEnd('v','q');
            // xf_acc = lazykinoPRM.pathstateSets[idx].polytrajEnd('a','x');
            // yf_acc = lazykinoPRM.pathstateSets[idx].polytrajEnd('a','y');
            // qf_acc = lazykinoPRM.pathstateSets[idx].polytrajEnd('a','q');
            // #######################################################################################

            for (int i = 0; i < int(xtraj.size()); i++)
            {
                geometry_msgs::Point p;
                p.x = xtraj[i];
                p.y = ytraj[i];
                p.z = DEFAULT_HIGH;
                Line.points.push_back(p);
                Line.id = marker_id;
            }
            // LineArray.markers.push_back(Line);

            // #######################################################################################
            //  Vector3d _position = lazykinoPRM.pathstateSets[idx].Position;
            //  cout << GREEN << "<------------------PathNodeState=" << idx << "------------------>" << RESET << endl;
            //  ROS_INFO("[\033[34mvisTraLibrary\033[0m]pathPosition =[%f,%f,%f]", _position(0), _position(1), _position(2));
            //  ROS_INFO("[\033[34mvisTraLibrary\033[0m]referT =[%f]", lazykinoPRM.pathstateSets[idx].referT);
            //  ROS_INFO("[\033[34mvisTraLibrary\033[0m]angledelta =[%f]", lazykinoPRM.pathstateSets[idx].angledelta);
            //  ROS_INFO("[\033[34mvisTraLibrary\033[0m]distance =[%f]", lazykinoPRM.pathstateSets[idx].distance);
            //  ROS_INFO("[\033[34mvisTraLibrary\033[0m]angle_cost  =[%f]",lazykinoPRM.pathstateSets[idx].angle_cost);
            //  ROS_INFO("[\033[34mvisTraLibrary\033[0m]trajectory_cost  =[%f]",lazykinoPRM.pathstateSets[idx].trajectory_cost);
            //  ROS_INFO("[\033[34mvisTraLibrary\033[0m]trajectory_length  =[%f]",lazykinoPRM.pathstateSets[idx].trajectory_length);
            //  cout << CYAN << "#########################################################################" << RESET << endl;
            //  ROS_INFO("[\033[34mvisTraLibrary\033[0m]spi_x  = %f, spi_y  = %f, spi_q  = %f", spi(0), spi(1), spi(2));
            //  ROS_INFO("[\033[34mvisTraLibrary\033[0m]xi_pos = %f, yi_pos = %f, qi_pos = %f", xi_pos, yi_pos, qi_pos);
            //  ROS_INFO("[\033[34mvisTraLibrary\033[0m]spf_x  = %f, spf_y  = %f, spf_q  = %f", spf(0), spf(1), spf(2));
            //  ROS_INFO("[\033[34mvisTraLibrary\033[0m]xf_pos = %f, yf_pos = %f, qf_pos = %f", xf_pos, yf_pos, qf_pos);
            //  cout << CYAN << "#########################################################################" << RESET << endl;
            //  ROS_INFO("[\033[34mvisTraLibrary\033[0m]svi_x  = %f, svi_y  = %f, svi_q  = %f", svi(0), svi(1), svi(2));
            //  ROS_INFO("[\033[34mvisTraLibrary\033[0m]xi_vel = %f, yi_vel = %f, qi_vel = %f", xi_vel, yi_vel, qi_vel);
            //  ROS_INFO("[\033[34mvisTraLibrary\033[0m]svf_x  = %f, svf_y  = %f, svf_q  = %f", svf(0), svf(1), svf(2));
            //  ROS_INFO("[\033[34mvisTraLibrary\033[0m]xf_vel = %f, yf_vel = %f, qf_vel = %f", xf_vel, yf_vel, qf_vel);
            //  cout << CYAN << "#########################################################################" << RESET << endl;
            //  ROS_INFO("[\033[34mvisTraLibrary\033[0m]sai_x  = %f, sai_y  = %f, sai_q  = %f", sai(0), sai(1), sai(2));
            //  ROS_INFO("[\033[34mvisTraLibrary\033[0m]xi_acc = %f, yi_acc = %f, qi_acc = %f", xi_acc, yi_acc, qi_acc);
            //  ROS_INFO("[\033[34mvisTraLibrary\033[0m]saf_x  = %f, saf_y  = %f, saf_q  = %f", saf(0), saf(1), saf(2));
            //  ROS_INFO("[\033[34mvisTraLibrary\033[0m]xf_acc = %f, yf_acc = %f, qf_acc = %f", xf_acc, yf_acc, qf_acc);
            // #######################################################################################
        }
        LineArray.markers.push_back(Line);
        ++marker_id;

        Spheres.id = marker_id;

        // cout << GREEN << "#########################################################################" << RESET << endl;

        // node of path

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
        ROS_DEBUG("[visTraLibrary]LineArray.size() = %d", (int)LineArray.markers.size());
        ROS_DEBUG("[visTraLibrary]marker_id = %d", marker_id);
    }
    _path_vis_pub.publish(LineArray);
}


void visRobotNode(const nav_msgs::Odometry::ConstPtr& msg)
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
    Body.ns              =  Line.ns              = "planner_node/visRobotNode";
    Body.action          =  Line.action          = visualization_msgs::Marker::ADD;

    Spheres.header.frame_id = "world";
    Spheres.header.stamp    = ros::Time::now();
    Spheres.ns              = "planner_node/visRobotNode";
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
    Body.scale.x         = robot_length;
    Body.scale.y         = robot_width;
    Body.scale.z         = robot_height;

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
    pt.x = pose.position.x - _local_w;
    pt.y = pose.position.y - _local_w;
    pt.z = DEFAULT_HIGH;
    Line.points.push_back(pt);
    pt.x = pose.position.x + _local_w;
    pt.y = pose.position.y - _local_w;
    pt.z = DEFAULT_HIGH;
    Line.points.push_back(pt);
    pt.x = pose.position.x + _local_w;
    pt.y = pose.position.y + _local_w;
    pt.z = DEFAULT_HIGH;
    Line.points.push_back(pt);
    pt.x = pose.position.x - _local_w;
    pt.y = pose.position.y + _local_w;
    pt.z = DEFAULT_HIGH;
    Line.points.push_back(pt);
    pt.x = pose.position.x - _local_w;
    pt.y = pose.position.y - _local_w;
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
    Spheres.pose.position.x = _localPose(0);
    Spheres.pose.position.y = _localPose(1);
    Spheres.pose.position.z = DEFAULT_HIGH;

    LineArray.markers.push_back(Spheres);

    _robot_vis_pub.publish(LineArray);
}

