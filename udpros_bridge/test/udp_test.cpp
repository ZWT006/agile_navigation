/*
 * @Author: wentao zhang && zwt190315@163.com
 * @Date: 2023-08-01
 * @LastEditTime: 2023-08-01
 * @Description: test udp communication
 * @reference: 
 * 
 * Copyright (c) 2023 by wentao zhang, All Rights Reserved. 
 */

#include <iostream>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Path.h>

#include <udpros_bridge/UDPbridge.h>

UDPBridge udp_bridge;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "udp_port");
    //创建句柄（虽然后面没用到这个句柄，但如果不创建，运行时进程会出错）
    ros::NodeHandle n;
 
    // udp_bridge.setUDP();
    //1  创建socket套接字
    int fd = socket(AF_INET, SOCK_DGRAM, 0);
    if(fd == -1)
    {
        perror("socket error");
        exit(1);
    }
    // fd绑定本地的IP和端口
    struct sockaddr_in serv;
    memset(&serv, 0, sizeof(serv));
    serv.sin_family = AF_INET;
    serv.sin_port = htons(1024);//设置服务器端口号
    serv.sin_addr.s_addr = htonl(INADDR_ANY);//获取本机ip
     //3.bind()绑定
    //参数一：0的返回值（fd）
    //参数二：(struct sockaddr*)&addr 前面结构体，即地址
    //参数三: addr结构体的长度
    int ret = bind(fd, (struct sockaddr*)&serv, sizeof(serv));
    if(ret == -1)
    {
        perror("bind error");
        exit(1);
    }
 
    struct sockaddr_in client;
    socklen_t cli_len = sizeof(client);
    // 通信
    char buf[1024] = {0}; //创建接收数据的数组
 
    ros::Rate loop_rate(50);//while以50Hz进行循环
 
    while(ros::ok()) 
    {
        int recvlen = recvfrom(fd, buf, sizeof(buf), 0, 
                               (struct sockaddr*)&client, &cli_len);//接收数据保存到buf,并返回数据长度。
        if(recvlen == -1)
        {
            perror("recvform error");
            exit(1);
        }
        printf("received data: %s\n", buf);
        char ip[64] = {0};
        printf(" Client IP: %s, Port: %d\n",
        inet_ntop(AF_INET, &client.sin_addr.s_addr, ip, sizeof(ip)),
        ntohs(client.sin_port));
 
        // 给客户端发送数据
        sendto(fd, buf, strlen(buf)+1, 0, (struct sockaddr*)&client, sizeof(client));
 
         ros::spinOnce(); 
        loop_rate.sleep(); 
    }
    close(fd);
    return 0;
}