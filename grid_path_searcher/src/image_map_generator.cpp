/*
 * @Author: wentao zhang && zwt190315@163.com
 * @Date: 2023-03-02
 * @LastEditTime: 2023-08-16
 * @Description: read picture to generate pointcloudmap
 * @reference: none
 * 
 * Copyright (c) 2023 by wentao zhang, All Rights Reserved. 
 */

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/kdtree.h>
#include <pcl/search/impl/kdtree.hpp>

#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Eigen>
#include <math.h>
#include <random>

//[ERROR] [1682305611.510883603, 5.800000000]: [ImageNode]:odom out of map range

// c++ 断言
#include <assert.h>

#define DEBUG_BUILD false
#define DEFAULT_HIGH 0.3f

#include <opencv2/opencv.hpp>

using namespace std;
using namespace Eigen;

using namespace cv;

ros::Publisher _global_map_pub,_local_map_pub,_erode_map_pub;
ros::Publisher _odometry_pub;
ros::Subscriber _odom_sub;

int pixe_grid,_local_w_grid;
double _x_size, _y_size, _z_size, _init_x, _init_y, _resolution, _sense_rate;
double _x_orign, _y_orign, _z_orign;
double _local_w;
string _image_address = string("/home/zwt/catkin_ws/src/grid_path_searcher/map/map1.png");

double _init_orign_x, _init_orign_y;

bool _set_init_state = false;
bool _has_map  = false;
bool erode_switch = false;

Mat map_img;
sensor_msgs::PointCloud2 globalMap_pcd;
sensor_msgs::PointCloud2 localMap_pcd;
sensor_msgs::PointCloud2 erodeMap_pcd;
pcl::PointCloud<pcl::PointXYZ> globalMap;
pcl::PointCloud<pcl::PointXYZ> localMap;
pcl::PointCloud<pcl::PointXYZ> erodeMap;

Mat erode_img;
Mat element;            // element for dilate and erode
int erode_kernel_size_;

pcl::search::KdTree<pcl::PointXYZ> kdtreeMap;
vector<int>     pointIdxSearch;
vector<float>   pointSquaredDistance;      


nav_msgs::Odometry odom_msg;
bool _has_odom = false;

void odomCallback(const nav_msgs::Odometry::ConstPtr & odom);
void LocalMapGenerate();
void GlobalMapGenerate();


void odomCallback(const nav_msgs::Odometry::ConstPtr & odom)
{
   
   odom_msg = *odom;
   _has_odom = true;

   //######################################################################
   // uint16_t img_x,img_y,idx,idy;
   // pcl::PointXYZ pt_image;
   
   // // one pixel is 0.01m, center is odom(x,y) for img(x,y)
   // img_x = odom->pose.pose.position.x + _x_orign;
   // img_y = odom->pose.pose.position.y + _y_orign;
   // img_x = img_x / 0.01;
   // img_y = img_y / 0.01;


   // if(img_x - _local_w_grid < 0 || img_y - _local_w_grid < 0 || img_x + _local_w_grid > map_img.cols || img_y +_local_w_grid > map_img.rows)
   // {
   //    ROS_ERROR("[ImageNode]:odom out of map range");
   //    return;
   // }
   
   // for (idx = img_x - _local_w_grid; idx < img_x + _local_w_grid; idx=idx+pixe_grid)
   // {
   //    for (idy = img_y - _local_w_grid; idy < img_y + _local_w_grid; idy=idy+pixe_grid)
   //    {
   //       // 如果该点有像素值 就是point, 使用Point()函数访问图像的像素值
   //       if (map_img.at<uchar>(Point(idx,idy)) == 0)
   //       {
   //          // for (int idz = 0; idz < _z_size / _resolution; idz++)
   //          // {

   //             pt_image.x = idx * 0.01 - _x_orign;
   //             pt_image.y = idy * 0.01 - _y_orign;
   //             pt_image.z = DEFAULT_HIGH;
   //             localMap.points.push_back(pt_image);
   //          // }
   //       }
   //    }
   // }

   // //odometry from map
   // odom_msg.header.stamp = ros::Time::now();
   // odom_msg.header.frame_id = "world";
   // _odometry_pub.publish(odom_msg);

   // //local map point clouds
   // localMap.width = localMap.points.size();
   // localMap.height = 1;
   // localMap.is_dense = true;
   // localMap_pcd.data.clear();
   // pcl::toROSMsg(localMap, localMap_pcd);
   // localMap_pcd.header.stamp = ros::Time::now();
   // localMap.header.frame_id = "world";
   // _local_map_pub.publish(localMap_pcd);
   // ROS_INFO("[\033[34mImageNode\033[0m]:odom:x = %2.4f,y = %2.4f",odom->pose.pose.position.x,odom->pose.pose.position.y);
   // ROS_INFO("[\033[34mImageNode\033[0m]:point cloud size: %d",localMap.points.size());
   // ROS_INFO("[\033[34mImageNode\033[0m]:-img_x = %d,+img_x = %d,-img_y = %d,+img_y = %d",img_x - _local_w_grid,img_x + _local_w_grid,img_y - _local_w_grid,img_y + _local_w_grid);
   // localMap.points.clear();
   //######################################################################
   //global map point clouds
   // globalMap_pcd.header.stamp = ros::Time::now();
   // _global_map_pub.publish(globalMap_pcd);
}


int main (int argc, char** argv) 
{        
   ros::init (argc, argv, "image_map");
   ros::NodeHandle n( "~" );
   //  ## 这句给出调试显示的级别为debug info 
   ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);

   _global_map_pub   = n.advertise<sensor_msgs::PointCloud2>("global_map", 1);
   _local_map_pub    = n.advertise<sensor_msgs::PointCloud2>("local_map", 1);
   _erode_map_pub    = n.advertise<sensor_msgs::PointCloud2>("erode_map", 1);
   _odometry_pub     = n.advertise<nav_msgs::Odometry>("/odometry", 1);
   _odom_sub         = n.subscribe<nav_msgs::Odometry>("odom", 1, odomCallback);
   // _odom_sub         = n.subscribe<nav_msgs::Odometry>("/ground_truth/state", 1, odomCallback);


//    n.param("init_state_x", _init_x,       0.0);
//    n.param("init_state_y", _init_y,       0.0);
   n.param("map/init_orign",  _set_init_state, false);
   n.param("map/orign_x",  _init_orign_x, 0.0);
   n.param("map/orign_y",  _init_orign_y, 0.0);

   n.param("map/x_size",  _x_size, 50.0);
   n.param("map/y_size",  _y_size, 50.0);
   n.param("map/z_size",  _z_size, 1.0 );

   n.param("map/orign_x_size",  _x_orign, 25.0);
   n.param("map/orign_y_size",  _y_orign, 25.0);
   n.param("map/orign_z_size",  _z_orign, 0.5 );

   n.param("map/resolution", _resolution, 0.05);
   n.param("map/local_w", _local_w, 2.0);
   n.param("map/erode_kernel_size", erode_kernel_size_, 3);
   n.param("map/erode_switch",erode_switch,false);

   n.param("sensing/rate", _sense_rate, 1.0);

   n.param("image_address",_image_address,string("map.png"));

   pixe_grid = _resolution*100;
   _local_w_grid = _local_w/0.01;
   element = getStructuringElement(cv::MORPH_RECT, cv::Size(erode_kernel_size_, erode_kernel_size_));
   ROS_DEBUG("Began picture to pcl");
   ROS_INFO("[\033[34mImageNode\033[0m]:image size: x = %2.4f,y = %2.4f",_x_size,_y_size);

   GlobalMapGenerate();
   ros::Rate loop_rate(_sense_rate);
   while (ros::ok())
   {
      globalMap_pcd.header.stamp = ros::Time::now();
      _global_map_pub.publish(globalMap_pcd);
      if (erode_switch)
      {
         erodeMap_pcd.header.stamp = ros::Time::now();
         _erode_map_pub.publish(erodeMap_pcd);
      }
      // ROS_INFO("[\033[34mImageNode\033[0m]:global map pub");
      // spinOnce 必须得接收到消息才会继续执行？
      ros::spinOnce();
      if(_has_odom)
      {
         LocalMapGenerate();
         _has_odom = false;
      }
      loop_rate.sleep();
   }
   ros::shutdown();
}

int localmap_cnt = 0;
void LocalMapGenerate()
{
   int idx,idy;
   double img_x,img_y;
   pcl::PointXYZ pt_image;
   
   // one pixel is 0.01m, center is odom(x,y) for img(x,y)
   img_x = odom_msg.pose.pose.position.x + _x_orign;
   img_y = odom_msg.pose.pose.position.y + _y_orign;
   img_x = img_x / 0.01;
   img_y = img_y / 0.01;

   int img_x_low = floor(img_x - _local_w_grid);
   int img_x_high = ceil(img_x + _local_w_grid);
   int img_y_low = floor(img_y - _local_w_grid);
   int img_y_high = ceil(img_y + _local_w_grid);

   if( img_x_low < 0 || img_y_low < 0 || img_x_high > map_img.cols || img_y_high > map_img.rows)
   {
      ROS_ERROR("[ImageNode]:odom out of map range");
      // if (img_x_low < 0)
      //    {img_x_low = 0;}
      // if (img_x_high > map_img.cols)
      //    {img_x_high = map_img.cols;}
      // if (img_y_low < 0)
      //    {img_y_low = 0;}
      // if (img_y_high > map_img.rows)
      //    {img_y_high = map_img.rows;}
      // if ()
      // return;
   }
   localMap.points.clear();
   // ROS_DEBUG("inage size: %d,%d",map_img.cols,map_img.rows);
   // ROS_DEBUG("img_x:%f,img_y:%f",img_x,img_y);
   // ROS_DEBUG("map size: -x:%d,+x:%d,-y:%d,+y%d",floor(img_x - _local_w_grid),ceil(img_x + _local_w_grid),floor(img_y - _local_w_grid),ceil(img_y + _local_w_grid));
   for (idx = floor(img_x - _local_w_grid); idx <= ceil(img_x + _local_w_grid); idx = idx + pixe_grid)
   {
      for (idy = floor(img_y - _local_w_grid); idy < ceil(img_y + _local_w_grid); idy = idy + pixe_grid)
      {
         if( idx < 0 || idy < 0 || idx > map_img.cols || idy > map_img.rows)
            continue;
         // 如果该点有像素值 就是point, 使用Point()函数访问图像的像素值
         if (map_img.at<uchar>(Point(idx,idy)) == 0)
         {
            pt_image.x = idx * 0.01 - _x_orign;
            pt_image.y = idy * 0.01 - _y_orign;
            pt_image.z = DEFAULT_HIGH + 0.05;
            localMap.points.push_back(pt_image);
         }
      }
   }
   // ROS_DEBUG("idx:%d,idy:%d",idx,idy);
   //odometry from map
   // odom_msg.header.stamp = ros::Time::now();
   // odom_msg.header.frame_id = "world";
   // _odometry_pub.publish(odom_msg);

   //local map point clouds
   localMap.width = localMap.points.size();
   localMap.height = 1;
   localMap.is_dense = true;
   localMap_pcd.data.clear();
   pcl::toROSMsg(localMap, localMap_pcd);
   localMap_pcd.header.stamp = ros::Time::now();
   localMap_pcd.header.frame_id = "world";
   _local_map_pub.publish(localMap_pcd);
   localmap_cnt++;
   if (localmap_cnt > 10){
      localmap_cnt = 0;
      ROS_INFO("[\033[34mImageNode\033[0m]:odom:x = %2.4f,y = %2.4f,pcl = %ld",odom_msg.pose.pose.position.x,odom_msg.pose.pose.position.y,localMap.points.size());
   }
   // ROS_INFO("[\033[34mImageNode\033[0m]:point cloud size: %d",localMap.points.size());
   // ROS_INFO("[\033[34mImageNode\033[0m]:-img_x = %d,+img_x = %d,-img_y = %d,+img_y = %d",img_x - _local_w_grid,img_x + _local_w_grid,img_y - _local_w_grid,img_y + _local_w_grid);
   localMap.points.clear();
}


void GlobalMapGenerate()
{  
   pcl::PointXYZ pt_image;

   // step1: read gary image
   // Load the image
   Mat orign_img = imread(_image_address,IMREAD_GRAYSCALE);
   if (orign_img.empty())
   {
      ROS_ERROR("image could not load !!!");
      ros::shutdown();
   }
   //读取图像二值化
   cv::threshold(orign_img, map_img, 128, 255,THRESH_BINARY);

   uint16_t img_x,img_y,idx,idy;
   // img_x = map_img.rows;
   // img_y = map_img.cols;
   /*The rows and cols of a cv::Mat object refer to its dimensions, 
   * specifically the number of rows and columns of pixels in the image, respectively. 
   * The number of rows represents the height of the image, 
   * while the number of columns represents its width.
   * cv::Mat object with 480 rows and 640 columns would represent a 480x640 pixel image.
   */
   img_x = map_img.cols;
   img_y = map_img.rows;
   if (_set_init_state) {
    _x_orign = _init_orign_x;
    _y_orign = _init_orign_y;
   }
   else {
    _x_orign = img_x*0.01/2;
    _y_orign = img_y*0.01/2;
   }

   if (DEBUG_BUILD)
   {
      imshow("show map image", map_img);
      waitKey(0);
      imshow("show orign image", orign_img);
      waitKey(0);
   }

   
   ROS_DEBUG("image rows: %d",img_y);
   ROS_DEBUG("image cols: %d",img_x);
   ROS_DEBUG("map_orign:x = %f,y = %f",_x_orign,_y_orign);

//######################################################################
// one pixel is 0.01m

// sing layer
   // for (idx = 0; idx < img_x; idx=idx+pixe_grid)
   // {
   //    for (idy = 0; idy < img_y; idy=idy+pixe_grid)
   //    {
   //       // 如果该点有像素值 就是point
   //       if (map_img.at<uchar>(idx, idy) == 0)
   //       {
   //          pt_image.x = idx * 0.01 - _x_orign;
   //          pt_image.y = idy * 0.01 - _y_orign;
   //          pt_image.z = 0.01;
   //          globalMap.points.push_back(pt_image);
   //       }
   //    }
   // }
// multiple layer
   for (idx = 0; idx < img_x; idx=idx+pixe_grid)
   {
      for (idy = 0; idy < img_y; idy=idy+pixe_grid)
      {
         // 如果该点有像素值 就是point, 使用Point()函数访问图像的像素值
         if (map_img.at<uchar>(Point(idx,idy)) == 0)
         {
            // for (int idz = 0; idz < _z_size / _resolution; idz++)
            // {

               pt_image.x = idx * 0.01 - _x_orign;
               pt_image.y = idy * 0.01 - _y_orign;
               pt_image.z = DEFAULT_HIGH;
               globalMap.points.push_back(pt_image);
            // }
         }
      }
   }

   ROS_DEBUG("PointClout size: %ld",globalMap.points.size());
   bool is_kdtree_empty = false;
   if(globalMap.points.size() > 0)
      kdtreeMap.setInputCloud( globalMap.makeShared() ); 
   else
      is_kdtree_empty = true;

   globalMap.width = globalMap.points.size();
   globalMap.height = 1;
   globalMap.is_dense = true;

   _has_map = true;
   
   pcl::toROSMsg(globalMap, globalMap_pcd);
   globalMap_pcd.header.frame_id = "world";

   if (erode_switch)
   {
      // erode
      erode(map_img, erode_img, element);
      // erode point clouds
      for (idx = 0; idx < img_x; idx=idx+pixe_grid)
      {
         for (idy = 0; idy < img_y; idy=idy+pixe_grid)
         {
            // 如果该点有像素值 就是point, 使用Point()函数访问图像的像素值
            if (erode_img.at<uchar>(Point(idx,idy)) == 0)
            {
               // for (int idz = 0; idz < _z_size / _resolution; idz++)
               // {

                  pt_image.x = idx * 0.01 - _x_orign;
                  pt_image.y = idy * 0.01 - _y_orign;
                  pt_image.z = DEFAULT_HIGH - 0.05;
                  erodeMap.points.push_back(pt_image);
               // }
            }
         }
      }
      erodeMap.width = erodeMap.points.size();
      erodeMap.height = 1;
      erodeMap.is_dense = true;
      pcl::toROSMsg(erodeMap, erodeMap_pcd);
      erodeMap_pcd.header.frame_id = "world";
   }
}