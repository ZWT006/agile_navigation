/*
 * @Author: wentao zhang && zwt190315@163.com
 * @Date: 2024-02-27
 * @LastEditTime: 2024-02-27
 * @Description: 
 * @reference: 
 * 
 * Copyright (c) 2024 by wentao zhang, All Rights Reserved. 
 */

#include <thread>
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>

nav_msgs::Odometry fakerobot;

ros::Publisher pubodom;
ros::Subscriber subnavs;

void rcvNavsCallback(const nav_msgs::Path::ConstPtr& msgs);

int main(int argc, char** argv)
{
    ros::init(argc, argv, "fakerobot_node");
    ros::NodeHandle nh("~");

    pubodom = nh.advertise<nav_msgs::Odometry>("/odometry", 1, true);
    subnavs = nh.subscribe("/nav_seq_vis",1,rcvNavsCallback);
    fakerobot.header.frame_id = "planner";
    fakerobot.pose.pose.position.x = 0;
    fakerobot.pose.pose.position.y = 0;
    fakerobot.pose.pose.position.z = 0;
    fakerobot.pose.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);

    ROS_INFO("fakerobot node: start");
    
    ros::Rate loop_rate(10);

    while(ros::ok()) 
    {
        ros::spinOnce();
        fakerobot.header.stamp = ros::Time::now();
        pubodom.publish(fakerobot);
        ROS_INFO("fakerobot pose: [%2.4f,%2.4f,%2.4f]",
        fakerobot.pose.pose.position.x,
        fakerobot.pose.pose.position.y,
        tf::getYaw(fakerobot.pose.pose.orientation));

        loop_rate.sleep();
    }
    // ros::spin();
    // ros::shutdown();
    return 0;
}

void rcvNavsCallback(const nav_msgs::Path::ConstPtr& msgs) 
{
    fakerobot.pose.pose.position = msgs->poses[1].pose.position;
    fakerobot.pose.pose.orientation = msgs->poses[1].pose.orientation;
}