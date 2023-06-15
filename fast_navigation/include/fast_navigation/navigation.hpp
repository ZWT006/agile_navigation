#ifndef _NAVIGATION_ASTAR_H
#define _NAVIGATION_ASTAR_H


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
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

#define DIST_XY 0.03f
#define DIST_Q 0.0872646f
// judge currend odom is near goal pose
// inline bool nearPose(Vector2d currPose,Vector2d goalPose,double currq,double goalq);
// double AngleMinDelta(double _start, double _goal);

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

#endif