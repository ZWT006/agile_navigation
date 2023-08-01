/*
 * @Author: wentao zhang && zwt190315@163.com
 * @Date: 2023-07-22
 * @LastEditTime: 2023-08-01
 * @Description: UDP communication bridge between ROS and other data source
 * @reference: 
 * @function lists:
 * 1. cv::Mat (CV_8UC1) <==> socket (UDP)       brief: obstacle map     reference: none
 * 2. geometry_msgs::Pose <==> socket (UDP)     brief: odometry/orientation   reference: http://docs.ros.org/en/api/geometry_msgs/html/msg/Pose.html
 * 3. nav_msgs::Path Path <==> socket (UDP)     brief: trajectory       reference: none
 * Copyright (c) 2023 by wentao zhang, All Rights Reserved. 
 */

#ifndef UDP_BRIDGE_H
#define UDP_BRIDGE_H

#include <iostream>
#include <string>
#include <math.h>
#include <vector>
#include <opencv2/opencv.hpp>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

////UDP communication parameters
#define BUFFER_SIZE_MAX 2048
#define BUFFER_SIZE 1024
#define UDP_HEAD 0xFF
#define UDP_END 0xEE
#define PACKAGE_HEAD 0xDD
// #define PACKAGE_END 0xCC
#define HEADER_SIZE 4 // header + length + struct_num = 4 bytes

//////  for test
// #define DEST_PORT 1024
// #define DSET_IP_ADDRESS  "192.168.123.12"


//  char datas structure for UDP communication #################################################
/* header:      1 bytes | 0xFF      |
 * length:      2 bytes | uint16_t  | length of all data from header to end
 * struct num:  1 bytes | uint8_t
 * PACKAGE_HEAD 1 bytes | 0xDD      | fast check
 * struct 0:    x bytes | data
 * PACKAGE_HEAD 1 bytes | 0xDD      | fast check
 * struct 1:    x bytes | data
 * ...
 * PACKAGE_HEAD 1 bytes | 0xDD      | fast check
 * struct n:    x bytes | data
 * end:         1 bytes | 0xEE      | necessary yes  -> fast check
 * struct -----------------------------------------------------------------------------------
 * data type:   2 bytes | uint16_t   | OBSTACLE_MAP, ROBOT_POSE, ROBOT_TRAJECTORY
 * data size:   2 bytes | uint16_t   | size of data
 * data:        x bytes | data       | data
 * struce -> data  ###########################################################################
 * cv::Mat binary_image_ struct -> data ------------------------------------------------------
 * cols        2 bytes | uint16_t   | cols of cv::Mat
 * rows        2 bytes | uint16_t   | rows of cv::Mat
 * resolution  4 bytes | float      | resolution of cv::Mat
 * char        x bytes | data       | bit 0|1 of cv::Mat Binary image 0/255
 * geometry_msgs::Pose Pose_ struct -> data --------------------------------------------------
 * x           8 bytes | double     | x
 * y           8 bytes | double     | y
 * z           8 bytes | double     | z
 * qx          8 bytes | double     | qx
 * qy          8 bytes | double     | qy
 * qz          8 bytes | double     | qz
 * qw          8 bytes | double     | qw
 * nav_msgs::Path Path_ struct -> data -------------------------------------------------------
 * pose size   2 bytes | uint16_t   | size of pose
 * pose struce 56 bytes| double x 7 | pose
*/

enum UDP_DATA_TYPE
{
    OBSTACLE_MAP    = (1 << 0),
    GEOMETRY_POSE   = (1 << 1),
    NAV_MSGS_PATH   = (1 << 2),
};


struct UDPHeader
{
    uint8_t header;
    uint16_t length;
    uint8_t struct_num;
};// 4 bytes

struct udp_orientation
{
    double x;
    double y;
    double z;
    double qx;
    double qy;
    double qz;
    double qw;
}; // 56 bytes

struct udp_struct_header
{
    uint16_t data_type;
    uint16_t data_size;
}; // 4 bytes

struct udp_struct_binary_map
{
    uint16_t cols;
    uint16_t rows;
    float resolution;
}; // 8 bytes

struct udp_struct_nav_msgs_path
{
    uint16_t pose_size;
}; // 2 bytes
/*
 *
*/
class UDPBridge
{
    private:
    int socket_fd_;
    int port_;
    std::string ip_;
    int DATA_TYPE_; // data type
    int buffer_size_;
    char *buffer_;  // buffer for data
    char send_buffer_[BUFFER_SIZE_MAX];
    char receive_buffer_[BUFFER_SIZE_MAX];
    int data_size_; // data size in bytes
    int data_index_;// current data index
    int struct_num_;// number of data struct
    //// struct bytes &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
    struct sockaddr_in addr_serv;
    //// datas
    cv::Mat binary_image_;
    float resolution_;
    uint16_t BinartImgSize_;
    bool BinartImgFlag;

    geometry_msgs::Pose Pose_;
    uint16_t PoseSize_;
    bool PoseFlag;

    nav_msgs::Path Path_;
    uint16_t PathSize_;
    bool PathFlag;

    inline bool unpackcvObsMat(char *data,const int data_size);
    inline bool unpackROSPose(char *data,const int data_size);
    inline bool unpackROSNavseq(char *data,const int data_size);

    public:
    UDPBridge() = default;
    ~UDPBridge() = default;
    bool setUDP(const int port,const char *ip);
    // bool setDataTypes(std::vector<UDP_DATA_TYPE> data_types);
    // bool setDataTypes(UDP_DATA_TYPE data_type);
    //TODO: add template for different data types
    bool resetsend();
    bool setHeaderandBail();
    bool send();
    bool receive();
    bool unpackStructData();


    bool setcvObsMat(const cv::Mat &mat,const float resolution);
    bool getcvObsMat(cv::Mat &mat,float &resolution);


    bool setROSPose(const geometry_msgs::Pose &pose);
    bool getROSPose(geometry_msgs::Pose &pose);


    bool setROSNavseq(const nav_msgs::Path &path);
    bool getROSNavseq(nav_msgs::Path &path);
};


#endif // UDP_BRIDGE_H