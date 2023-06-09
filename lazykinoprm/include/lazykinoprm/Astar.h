/*
 * @Author: wentao zhang && zwt190315@163.com
 * @Date: 2023-04-03
 * @LastEditTime: 2023-04-04
 * @Description: 
 * @reference: 
 * 
 * Copyright (c) 2023 by wentao zhang, All Rights Reserved. 
 */




#ifndef _ASTAR_H
#define _ASTAR_H

#include <Eigen/Eigen>
#include <iostream>
#include <map>
#include <ros/console.h>
#include <ros/ros.h>
#include <string>
#include <unordered_map>
// #include "grad_spline/sdf_map.h"
#include <boost/functional/hash.hpp>
#include <queue>
//project .h
#include <GridNode.h>
#include <State.h>

using namespace std;
using namespace Eigen;

#define NONOPEN 9999

//A star graph search node
struct AstarNode;
typedef AstarNode* AstarNodePtr;

struct AstarNode
{
  Vector3d pose;
  Vector3i index;
  uint32_t listidx;
  NodeState state;
  double pathCost,heurtCost,fnCost; // fn = pathCost + heurtCost;
};

//A star graph search class
class OpenList
{
protected:
  // C++ https://en.cppreference.com/w/cpp/container/multimap for quick indexing
  //  but it's not suitable this condition, because GridNode is change in searching process
  //  their states is different
  //  GridNodePtr terminatePtr;
  //  std::multimap<double, GridNodePtr> openset;
  uint32_t list_length;               // open list length
  vector<Eigen::Vector3d> node_pose;  // node pose list
  vector<Eigen::Vector3i> node_index; // node index list
  vector<uint32_t> node_listidx;      // node index in openlist for for quick indexing
  // vector<Eigen::Vector3i> parent_index; //parent index list
  vector<uint32_t> parent_listidx; // parent node index in openlist for for quick indexing
  vector<double> pathCostSets;     // openlist fn score list

  // std::multimap<double, uint32_t>::iterator fnlist;
  vector<NodeState> nodestateSets; // node state list
  // vector<AstarNode> nodesSets;  //node state list

  vector<double> fnCostSets; // openlist fn score list
  vector<uint32_t> fnlist;   // fnCostSets >> listidx

private:
  void InsertFnlist(double _fncost, uint32_t _listidx)
  {
    for (uint32_t idx = 0; idx < fnCostSets.size(); idx++)
    {
      if (fnCostSets[idx] >= _fncost)
      {
        // if false
        //  assert(fnlist[idx] != _listidx);
        if (fnlist[idx] == _listidx)
          ROS_ERROR("openlist fnlist insert is wrong");
        fnlist.insert(fnlist.begin() + idx, _listidx);
        fnCostSets.insert(fnCostSets.begin() + idx, _fncost);
        return;
      }
    }
    fnlist.insert(fnlist.end() + 1, _listidx);
    fnCostSets.insert(fnCostSets.end() + 1, _fncost);
  }
  void EraseFnlist(uint32_t _listidx)
  {
    for (uint32_t idx = 0; idx < fnlist.size(); idx++)
    {
      if (fnlist[idx] == _listidx)
      {
        // if false
        //  assert(fnlist[idx] != _listidx);
        if (fnlist[idx] == _listidx)
        {
          fnlist.erase(fnlist.begin() + idx);
          fnCostSets.erase(fnCostSets.begin() + idx);
          return;
        }
      }
    }
    // erase a list wrong the listidx inexistence
    ROS_ERROR("openlist erase fnlist is inexistence");
  }

public:
  // if Index is in openlist return it't listidx
  uint32_t isOpenList(Eigen::Vector3i _Index)
  {
    uint32_t _listidx = NONOPEN;
    Eigen::Vector3i temp_index;
    for (uint32_t idx = 0; idx < list_length; idx++)
    {
      temp_index = node_index[idx];
      if (temp_index == _Index)
      {
        _listidx = idx;
        return;
      }
    }
  }
  bool IsAllExtend()
  {
    return fnCostSets.empty();
  }
  AstarNode OpenListMinPop();
  // void insert(AstarNode _node_new, AstarNode _node_pare, NodeState _node_state);
  void insert(AstarNodePtr _node_new, AstarNodePtr _node_pare, NodeState _node_state);

// TODO
  OpenList()
  {
    list_length=0;
  }
  ~OpenList(){};
};

// class Astar
// {
  
//   public:
//     // get deflection angle
//   double getDeflection(Eigen::Vector3d Pose_new,Eigen::Vector3d Pose_pare)
//   {
//     // return deflection angle from Pose_pare to Pose_new
//     double angle = 0;
//     angle = atan2((Pose_new(2) - Pose_pare(2)), (Pose_new(1) - Pose_pare(1)));
//     return angle;
//   }
// };

#endif