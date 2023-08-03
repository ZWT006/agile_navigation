/*
 * @Author: wentao zhang && zwt190315@163.com
 * @Date: 2023-07-22
 * @LastEditTime: 2023-08-02
 * @Description: UDP communication bridge between ROS and other data source
 * @reference: 
 * @function lists:
 * 1. cv::Mat (CV_8UC1) <==> socket (UDP)       brief: obstacle map     reference: none
 * 2. geometry_msgs::Pose <==> socket (UDP)     brief: odometry/orientation   reference: http://docs.ros.org/en/api/geometry_msgs/html/msg/Pose.html
 * 3. nav_msgs::Path Path <==> socket (UDP)     brief: trajectory       reference: none
 * Copyright (c) 2023 by wentao zhang, All Rights Reserved. 
 */

#ifndef UDP_BRIDGE_HPP
#define UDP_BRIDGE_HPP

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
#define HEADER_SIZE sizeof(UDPHeader) // 6 bytes

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
    char header;
    uint16_t length;
    uint16_t struct_num;
};// 5 bytes

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
    public:
    int socket_fd_;
    int port_;
    std::string ip_;
    int send_flag_;
    int receive_flag_;
    char serveripAddress[INET_ADDRSTRLEN];
    uint16_t serverport;
    in_addr_t serveraddr;
    char clientipAddress[INET_ADDRSTRLEN];
    uint16_t clientport;
    in_addr_t clientaddr;
    int DATA_TYPE_; // data type
    int buffer_size_;
    char *buffer_;  // buffer for data
    char send_buffer_[BUFFER_SIZE_MAX];
    char receive_buffer_[BUFFER_SIZE_MAX];
    int data_size_; // data size in bytes
    int data_index_;// current data index
    int struct_num_;// number of data struct
    //// struct bytes &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
    struct sockaddr_in addr_server;
    struct sockaddr_in addr_client;
    struct sockaddr_in addr_local;
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
    bool setserverUDP(const int port,const char *ip);
    bool setclientUDP(const int port,const char *ip);
    bool setTimeout(int timeout_in_seconds);
    bool closeUDP();
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

bool UDPBridge::setUDP(const int port,const char *ip) {
    port_ = port;
    ip_ = std::string(ip);
    socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
    std::cout << "socket_fd_: " << socket_fd_ << std::endl;
    if(socket_fd_ < 0) {
        std::cout << "socket create failed" << std::endl;
        return false;
    }
    //将addr_serv数组全体内存空间按字节整体清零
    memset(&addr_local, 0, sizeof(addr_local));
    addr_local.sin_family = AF_INET;
    addr_local.sin_addr.s_addr = inet_addr(ip); //服务器ip，不是本机ip
    // std::cout << "inet_addr(ip): " << inet_addr(ip) << std::endl;
    
    inet_ntop(AF_INET, &addr_local.sin_addr.s_addr, serveripAddress, INET_ADDRSTRLEN);
    // std::cout << "inet_ntop: " << serveripAddress << std::endl;
    addr_local.sin_port = htons(port);//服务器端口号
    // std::cout << "htons(port): " << htons(port) << std::endl;
    int ret = bind(socket_fd_, (struct sockaddr*)&addr_local, sizeof(addr_local));
    if(ret == -1)
    {
        perror("bind error");
        return false;
    }
    return true;
}

bool UDPBridge::setserverUDP(const int port,const char *ip) {
    //将addr_serv数组全体内存空间按字节整体清零
    memset(&addr_server, 0, sizeof(addr_server));
    addr_server.sin_family = AF_INET;
    addr_server.sin_addr.s_addr = inet_addr(ip); //服务器ip，不是本机ip
    serveraddr = inet_addr(ip); 
    inet_ntop(AF_INET, &addr_server.sin_addr.s_addr, serveripAddress, INET_ADDRSTRLEN);
    addr_server.sin_port = htons(port);//服务器端口号
    serverport = htons(port);
    return true;
}

bool UDPBridge::setclientUDP(const int port,const char *ip) {
    //将addr_serv数组全体内存空间按字节整体清零
    memset(&addr_client, 0, sizeof(addr_client));
    addr_client.sin_family = AF_INET;
    addr_client.sin_addr.s_addr = inet_addr(ip); //服务器ip，不是本机ip
    clientaddr = inet_addr(ip); 
    inet_ntop(AF_INET, &addr_client.sin_addr.s_addr, clientipAddress, INET_ADDRSTRLEN);
    addr_client.sin_port = htons(port);//服务器端口号
    clientport = htons(port);
    return true;
}

bool UDPBridge::setTimeout(int timeout_in_seconds) {
    struct timeval timeout;
    timeout.tv_sec = timeout_in_seconds;
    timeout.tv_usec = 0;
    int ret = setsockopt(socket_fd_, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
    if (ret < 0)
        return false;
    return true;
}

bool UDPBridge::closeUDP() {
    close(socket_fd_);
    return true;
}

bool UDPBridge::resetsend() {
    memset(send_buffer_, 0, sizeof(send_buffer_));
    data_index_ = HEADER_SIZE;
    struct_num_ = 0;

    BinartImgSize_ = 0;
    PoseSize_ = 0;
    PathSize_ = 0;

    return true;
}

bool UDPBridge::setHeaderandBail() {
    send_buffer_[data_index_++] = UDP_END;
    UDPHeader header;
    header.header = UDP_HEAD;
    data_size_ = data_index_;
    header.length = data_size_;
    header.struct_num = struct_num_;
    memcpy(send_buffer_, &header, sizeof(header));
    return true;
}

bool UDPBridge::unpackStructData() {
    // 系统cpu默认数据位数会有影响 32位系统 64位系统 0xFF 是严格的8位 但是其余数据的计算和比较不同
    if (uint8_t(receive_buffer_[0] ^ UDP_HEAD)) // ^ 异或 直接比较数值出错 -1 != 255
        return false;
    int local_data_index = 0;
    UDPHeader header;
    memcpy(&header, receive_buffer_, sizeof(header));
    local_data_index += sizeof(header);
    data_size_ = header.length;
    struct_num_ = header.struct_num;
    if (uint8_t(receive_buffer_[data_size_ - 1] ^ UDP_END))
        return false;
    while (local_data_index < data_size_ - 1)
    {
        if (uint8_t(receive_buffer_[local_data_index] ^ PACKAGE_HEAD))
            return false;
        local_data_index++;
        udp_struct_header struct_header;
        memcpy(&struct_header, receive_buffer_ + local_data_index, sizeof(struct_header));
        local_data_index += sizeof(struct_header);
        if (local_data_index + struct_header.data_size > data_size_)
            return false;
        switch (struct_header.data_type)
        {
        case OBSTACLE_MAP:
            BinartImgFlag = unpackcvObsMat(receive_buffer_ + local_data_index, struct_header.data_size);
            break;
        case GEOMETRY_POSE:
            PoseFlag =  unpackROSPose(receive_buffer_ + local_data_index, struct_header.data_size);
            break;
        case NAV_MSGS_PATH:
            PathFlag = unpackROSNavseq(receive_buffer_ + local_data_index, struct_header.data_size);
            break;
        default:
            return false;
            break;
        }
        local_data_index += struct_header.data_size;
    }
    return true;
}
/*
1>函数原型：
#include<sys/types.h>
#include<sys/socket.h>
//ssize_t sendo(ints,const void *msg,size_t len,int flags,const struct sockaddr *to,socklen_ttolen);
2>函数功能：
向目标主机发送消息
3>函数形参：
Ø  s:套接字描述符。
Ø  *msg:发送缓冲区
Ø  len:待发送数据的长度
Ø  flags:控制选项，一般设置为0或取下面的值
(1)MSG_OOB:在指定的套接字上发送带外数据(out-of-band data),该类型的套接字必须支持带外数据（eg:SOCK_STREAM）.
(2)MSG_DONTROUTE:通过最直接的路径发送数据，而忽略下层协议的路由设置。
Ø  to:用于指定目的地址
Ø  tolen:目的地址的长度。
4>函数返回值：
执行成功后返回实际发送数据的字节数，出错返回-1，错误代码存入errno中。
*/
bool UDPBridge::send() {
    auto send_flag = sendto(socket_fd_, send_buffer_, data_size_, 0, (struct sockaddr *)&addr_server, sizeof(addr_server));
    send_flag_ = send_flag;
    if (send_flag < 0)
        return false;
    else
        return true;
}
/*
1>函数原型：
#include<sys/types.h>
#include<sys/socket.h>
ssize_t recvfrom(int s,void *buf,size_t len,intflags,struct sockaddr *from,socklen_t *fromlen);
2>函数功能：接收数据
3>函数形参：
Ø  int s:套接字描述符
Ø  buf:指向接收缓冲区，接收到的数据将放在这个指针所指向的内存空间。
Ø  len:指定了缓冲区的大小。
Ø  flags:控制选项,一般设置为0或取以下值
(1)MSG_OOB:请求接收带外数据
(2)MSG_PEEK:只查看数据而不读出
(3)MSG_WAITALL:只在接收缓冲区时才返回。
Ø  *from:保存了接收数据报的源地址。
Ø  *fromlen:参数fromlen在调用recvfrom前为参数from的长度，调用recvfrom后将保存from的实际大小。
4>函数返回值：
执行成功后返回实际接收到数据的字节数，出错时则返回-1，错误代码存入errno中。
*/
bool UDPBridge::receive() {
    socklen_t len = sizeof(addr_client);
    auto recv_flag = recvfrom(socket_fd_, receive_buffer_, BUFFER_SIZE_MAX, 0, (struct sockaddr *)&addr_client, &len);
    receive_flag_ = recv_flag;
    if (recv_flag < 0)
        return false;
    else
        return true;
}

bool UDPBridge::setcvObsMat(const cv::Mat &mat,const float resolution) {
    send_buffer_[data_index_++] = PACKAGE_HEAD;
    binary_image_ = mat.clone();

    udp_struct_header header;
    header.data_type = OBSTACLE_MAP;
    
    udp_struct_binary_map obstacle_map;
    obstacle_map.cols = binary_image_.cols;
    obstacle_map.rows = binary_image_.rows;
    obstacle_map.resolution = resolution;
    int matrix_row = binary_image_.rows ;
    int matrix_col = ceil(binary_image_.cols / 8.0);
    BinartImgSize_ = matrix_row * matrix_col;
    header.data_size = sizeof(udp_struct_binary_map) + BinartImgSize_;
    memcpy(send_buffer_ + data_index_, &header, sizeof(header));
    data_index_ += sizeof(header);
    memcpy(send_buffer_ + data_index_, &obstacle_map, sizeof(obstacle_map));
    data_index_ += sizeof(obstacle_map);

    for (int i = 0; i < matrix_row; i++)
    {
        for (int j = 0; j < matrix_col; j++)
        {
            char pixel = 0;
            for (int k = 0; k < 8 && (j * 8 + k) < binary_image_.cols; k++)
            {
                if (binary_image_.at<uchar>(i, j * 8 + k) == 255)
                {
                    pixel |= (1 << k);
                }
            }
            send_buffer_[data_index_++] = pixel;
        }
    }
    struct_num_++;

    return true;
}

inline bool UDPBridge::unpackcvObsMat(char *data,const int data_size) {
    int local_data_index = 0;
    udp_struct_binary_map obstacle_map;
    memcpy(&obstacle_map, data, sizeof(obstacle_map));
    local_data_index += sizeof(obstacle_map);
    resolution_ = obstacle_map.resolution;
    BinartImgSize_ = obstacle_map.cols * obstacle_map.rows;
    cv::Mat mat(obstacle_map.rows, obstacle_map.cols, CV_8UC1);
    int matrix_row = mat.rows;
    int matrix_col = ceil(mat.cols / 8.0);
    for (int i = 0; i < matrix_row; i++)
    {
        for (int j = 0; j < matrix_col; j++)
        {
            if (local_data_index >= data_size)
                return false;
            char pixel = data[local_data_index++];
            for (int k = 0; k < 8 && (j * 8 + k) < binary_image_.cols; k++) {
                if (pixel & (1 << k)) {
                    mat.at<uchar>(i, j * 8 + k) = 255;
                }
                else {
                    mat.at<uchar>(i, j * 8 + k) = 0;
                }
            }
        } 
    }
    binary_image_ = mat.clone();
    return true;
}

bool UDPBridge::getcvObsMat(cv::Mat &mat,float &resolution) {
    if (binary_image_.empty() || !BinartImgFlag)
        return false;
    mat = binary_image_.clone();
    resolution = resolution_;
    return true;
}

bool UDPBridge::setROSPose(const geometry_msgs::Pose &pose) {
    send_buffer_[data_index_++] = PACKAGE_HEAD;
    Pose_ = pose;
    udp_struct_header header;
    header.data_type = GEOMETRY_POSE;
    header.data_size = sizeof(udp_orientation);
    memcpy(send_buffer_ + data_index_, &header, sizeof(header));
    data_index_ += sizeof(header);
    udp_orientation orientation;
    orientation.x = Pose_.position.x;
    orientation.y = Pose_.position.y;
    orientation.z = Pose_.position.z;
    orientation.qx = Pose_.orientation.x;
    orientation.qy = Pose_.orientation.y;
    orientation.qz = Pose_.orientation.z;
    orientation.qw = Pose_.orientation.w;
    memcpy(send_buffer_ + data_index_, &orientation, sizeof(orientation));
    data_index_ += sizeof(orientation);
    struct_num_++;
    return true;
}

inline bool UDPBridge::unpackROSPose(char *data,const int data_size) {
    if (data_size != sizeof(udp_orientation))
        return false;
    udp_orientation orientation;
    memcpy(&orientation, data, sizeof(orientation));
    Pose_.position.x = orientation.x;
    Pose_.position.y = orientation.y;
    Pose_.position.z = orientation.z;
    Pose_.orientation.x = orientation.qx;
    Pose_.orientation.y = orientation.qy;
    Pose_.orientation.z = orientation.qz;
    Pose_.orientation.w = orientation.qw;
    PoseSize_ = data_size;
    return true;
}

bool UDPBridge::getROSPose(geometry_msgs::Pose &pose) {
    if (!PoseFlag)
        return false;
    pose = Pose_;
    return true;
}

bool UDPBridge::setROSNavseq(const nav_msgs::Path &path) {
    send_buffer_[data_index_++] = PACKAGE_HEAD;
    Path_ = path;
    udp_struct_header header;
    header.data_type = NAV_MSGS_PATH;
    header.data_size = sizeof(udp_struct_nav_msgs_path) + Path_.poses.size() * sizeof(udp_orientation);
    memcpy(send_buffer_ + data_index_, &header, sizeof(header));
    data_index_ += sizeof(header);
    udp_struct_nav_msgs_path nav_msgs_path;
    nav_msgs_path.pose_size = Path_.poses.size();
    memcpy(send_buffer_ + data_index_, &nav_msgs_path, sizeof(nav_msgs_path));
    data_index_ += sizeof(nav_msgs_path);
    for (int i = 0; i < Path_.poses.size(); i++)
    {
        udp_orientation orientation;
        orientation.x = Path_.poses[i].pose.position.x;
        orientation.y = Path_.poses[i].pose.position.y;
        orientation.z = Path_.poses[i].pose.position.z;
        orientation.qx = Path_.poses[i].pose.orientation.x;
        orientation.qy = Path_.poses[i].pose.orientation.y;
        orientation.qz = Path_.poses[i].pose.orientation.z;
        orientation.qw = Path_.poses[i].pose.orientation.w;
        memcpy(send_buffer_ + data_index_, &orientation, sizeof(orientation));
        data_index_ += sizeof(orientation);
    }
    struct_num_++;
    return true;
}

inline bool UDPBridge::unpackROSNavseq(char *data,const int data_size) {
    if (data_size < sizeof(udp_struct_nav_msgs_path))
        return false;
    udp_struct_nav_msgs_path nav_msgs_path;
    memcpy(&nav_msgs_path, data, sizeof(nav_msgs_path));
    PoseSize_ = nav_msgs_path.pose_size;
    int local_data_index = sizeof(udp_struct_nav_msgs_path);
    if (!Path_.poses.empty())
        Path_.poses.clear();
    for (int i = 0; i < PoseSize_; i++)
    {
        if (local_data_index + sizeof(udp_orientation) > data_size)
            return false;
        udp_orientation orientation;
        memcpy(&orientation, data + local_data_index, sizeof(orientation));
        local_data_index += sizeof(orientation);
        //reference: https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PoseStamped.html
        geometry_msgs::PoseStamped Posemsg;
        Posemsg.pose.position.x = orientation.x;
        Posemsg.pose.position.y = orientation.y;
        Posemsg.pose.position.z = orientation.z;
        Posemsg.pose.orientation.x = orientation.qx;
        Posemsg.pose.orientation.y = orientation.qy;
        Posemsg.pose.orientation.z = orientation.qz;
        Posemsg.pose.orientation.w = orientation.qw;
        Path_.poses.push_back(Posemsg);
    }
    return true;
}

bool UDPBridge::getROSNavseq(nav_msgs::Path &path) {
    if (!PathFlag || Path_.poses.empty())
        return false;
    path = Path_;
    return true;
}


#endif // UDP_BRIDGE_H