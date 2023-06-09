/*
 * @Author: wentao zhang && zwt190315@163.com
 * @Date: 2023-04-03
 * @LastEditTime: 2023-05-04
 * @Description: 
 * @reference: 
 * 
 * Copyright (c) 2023 by wentao zhang, All Rights Reserved. 
 */

#ifndef _STATE_H_
#define _STATE_H_

#include <iostream>
#include <ros/ros.h>
#include <ros/console.h>
#include <Eigen/Eigen>
// #include "backward.hpp"

#define INFO_K \033[30m
#define INFO_R \033[31m
#define INFO_G \033[32m
#define INFO_Y \033[33m
#define INFO_B \033[34m
#define INFO_P \033[35m
#define INFO_C \033[36m
#define INFO_H \033[1m
#define INFO_END \033[0m

struct TrajectoryState;
typedef TrajectoryState* TrajectoryStatePtr;

struct TrajectoryState
{
    std::vector<Eigen::Vector3d> Position;
    std::vector<Eigen::Vector3d> Velocity;
    double Trajctory_Cost ;
    bool collision_check ;           //False -> no collision, True -> collision
    bool optimal_flag;               //False -> not optimal in TraLibrary, True -> not optimal in TraLibrary, 

    TrajectoryState(std::vector<Eigen::Vector3d> _Position, std::vector<Eigen::Vector3d> _Velocity,double _Trajctory_Cost){
        Position        = _Position;
        Velocity        = _Velocity;
        Trajctory_Cost  = _Trajctory_Cost;
        collision_check = false;
        optimal_flag    = false;
    }
    TrajectoryState(){};
    ~TrajectoryState(){};

    void setCollisionfree(){
        collision_check = true;
    }
    void setOptimal(){
        optimal_flag    = true;
    }
};

#endif