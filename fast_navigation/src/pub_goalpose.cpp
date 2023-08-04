/*
 * @Author: wentao zhang && zwt190315@163.com
 * @Date: 2023-08-03
 * @LastEditTime: 2023-08-03
 * @Description: pub goal pose as nav_msgs::Path
 * @reference: 
 * 
 * Copyright (c) 2023 by wentao zhang, All Rights Reserved. 
 */

#include <thread>
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pub_goalpose");
    ros::NodeHandle nh("~");

    // 获取输入参数 x、y 和 q
    double target_x, target_y, target_q;
    if (argc != 4) {
        ROS_ERROR("Usage: pub_goalpose <x> <y> <q>");
        return 1;
    } else {
        target_x = atof(argv[1]);
        target_y = atof(argv[2]);
        target_q = atof(argv[3]);
    }
    double target_q_rad = target_q/180.0*M_PI;
    ROS_INFO("target_x: %2.4fm, target_y: %2.4fm, target_q: %2.4fdeg %2.4frad", target_x, target_y, target_q, target_q_rad);

    // while(ros::ok())
    // {
    // 创建路径消息
    nav_msgs::Path path_msg;
    path_msg.header.stamp = ros::Time::now();
    path_msg.header.frame_id = "world";

    // 创建路径上的位姿点
    geometry_msgs::PoseStamped pose;
    pose.header = path_msg.header;
    pose.pose.position.x = target_x;
    pose.pose.position.y = target_y;
    pose.pose.position.z = 0.0;
    pose.pose.orientation = tf::createQuaternionMsgFromYaw(target_q_rad);

    // 将位姿点加入到路径消息中
    path_msg.poses.push_back(pose);

    

    // ros::Publisher pub_goalpose = nh.advertise<nav_msgs::Path>("/goal_pose", 1);
    ros::Publisher pub_goalpose = nh.advertise<nav_msgs::Path>("/goal_pose", 1, true);
    pub_goalpose.publish(path_msg);

    // 延时一段时间以确保消息能够被发布
    std::this_thread::sleep_for(std::chrono::seconds(1));
    // ros::Duration(1.0).sleep(); 这个函数竟然不行，不知道为什么 程序就卡在这里了
    ROS_INFO("goal pose published");
    // }
    // 退出

    return 0;
}