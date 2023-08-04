/*
 * @Author: wentao zhang && zwt190315@163.com
 * @Date: 2023-04-03
 * @LastEditTime: 2023-07-06
 * @Description: 
 * @reference: 
 * 
 * Copyright (c) 2023 by wentao zhang, All Rights Reserved. 
 */

#ifndef _GRIDNODE_H_
#define _GRIDNODE_H_

#include <vector>
#include <iostream>
#include <Eigen/Eigen>

#include "Polynomial.h"

#define Naf 9999


// #define INFO_K \033[30m
// #define INFO_R \033[31m
// #define INFO_G \033[32m
// #define INFO_Y \033[33m
// #define INFO_B \033[34m
// #define INFO_P \033[35m
// #define INFO_C \033[36m
// #define INFO_H \033[1m
// #define INFO_END \033[0m

// typedef struct Score 
// {
  
// }ScoreStc;
// ScoreStc NodeScore;

//############################################################################
//node class
//Node Type list:Start -> S;Goal -> G;Midpoint -> M;Invalid -> W;Extend -> E;
enum GridNodeType_Set {Start,Goal,Mid,Invalid,Extend} GridNodeType;
struct GridNode;
typedef GridNode* GridNodePtr;

struct GridNode
{
// private:
  char type;             //  point type:
  Eigen::Vector3d pose;  // pose of gridnode  [x,y,θ]
  Eigen::Vector3d posi;  // position
  Eigen::Vector3d orit;  // oritention
  // Eigen::Vector3i dir;   // direction of expanding
  Eigen::Vector3i index; // pose_map index [rows,cols,deepth]
  // double gScore, fScore;
  // NodeScore
  // GridNodePtr parentPtr;
  // std::multimap<double, GridNodePtr>::iterator nodeMapIt;

  GridNode(Eigen::Vector3d _Pose, Eigen::Vector3i _Index,GridNodeType_Set _Type)
  {
    type = _Type;
    pose = _Pose;
    index = _Index;
    // type = Invalid;
    // pose = Eigen::Vector3d(0,0,0);
    // index = Eigen::Vector3i(0,0,0);
    // parentPtr = NULL;
    
    // gScore = Naf;
    // fScore = Naf;
    
  }
// public:
  GridNode(){};
  ~GridNode(){};
  // set Pose function
  void setPose(Eigen::Vector3d _Pose, Eigen::Vector3i _Index)
  {
    pose = _Pose;
    index = _Index;
  }
  // set Type function
  void setType(GridNodeType_Set _Type)
  {
    type = _Type;
  }
};

//############################################################################
//
//node state structure
struct NodeState;
typedef NodeState* NodeStatePtr;

struct NodeState
{
    Eigen::Vector3d Position;
    Eigen::Vector3d Velocity;
    Eigen::Vector3d Acceleration;
    double Trajectory_Cost;
    double Trajectory_Length;
    bool collision_check; // False -> no collision, True -> collision
    bool optimal_flag;    // False -> not optimal in TraLibrary, True -> not optimal in TraLibrary,

    NodeState(Eigen::Vector3d _Position, Eigen::Vector3d _Velocity, Eigen::Vector3d _Acceleration, double _Trajectory_Cost,double _Trajectory_Length)
    {
      Position = _Position;
      Velocity = _Velocity;
      Acceleration = _Acceleration;
      Trajectory_Cost = _Trajectory_Cost;
      Trajectory_Length = _Trajectory_Length;
      collision_check = false;
      optimal_flag = false;
    }
    NodeState(){};
    ~NodeState(){};

    void setCollisionfree()
    {
      collision_check = true;
    }
    void setOptimal()
    {
      optimal_flag = true;
    }
};

//############################################################################
//
//node state structure
struct PathState;
typedef PathState* PathStatePtr;
struct PathState
{
  Eigen::Vector3d spi;
  Eigen::Vector3d svi;
  Eigen::Vector3d sai;
  Eigen::Vector3d spf;
  Eigen::Vector3d svf;
  Eigen::Vector3d saf;
  double referT;
  double dt;
  double costJ;
  double trajlength;
  Polynomial poly;

  std::vector<double> xcoeff;
  std::vector<double> ycoeff;
  std::vector<double> qcoeff;

  PathState(){};
    ~PathState(){};

  // std::vector<double> xtraj(){};
  // std::vector<double> ytraj(){};
  // std::vector<double> qtraj(){};
  std::vector<double> xtraj()
  {
    std::vector<double> traj;
    std::vector<double> timeseq;
    timeseq = poly.getseqence(0.0,referT,dt);
    traj = poly.PolyvalVector(xcoeff,timeseq);
    return traj;
  }
  std::vector<double> ytraj()
  {
    std::vector<double> traj;
    std::vector<double> timeseq;
    timeseq = poly.getseqence(0.0,referT,dt);
    traj = poly.PolyvalVector(ycoeff,timeseq);
    return traj;
  }
  std::vector<double> qtraj()
  {
    std::vector<double> traj;
    std::vector<double> timeseq;
    timeseq = poly.getseqence(0.0,referT,dt);
    traj = poly.PolyvalVector(ycoeff,timeseq);
    return traj;
  }
};

#endif //_GRIDNODE_H_