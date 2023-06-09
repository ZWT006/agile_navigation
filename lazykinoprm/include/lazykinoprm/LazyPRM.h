/*
 * @Author: wentao zhang && zwt190315@163.com
 * @Date: 2023-04-03
 * @LastEditTime: 2023-05-04
 * @Description: 
 * @reference: 
 * 
 * Copyright (c) 2023 by wentao zhang, All Rights Reserved. 
 */


#ifndef _LAZY_PRM_H
#define _LAZY_PRM_H

#include <Eigen/Eigen>
#include <iostream>
#include <ros/console.h>
#include <ros/ros.h>
#include <string>
#include <unordered_map>
#include <queue>
#include <vector>
//project .h
#include <lazykinoprm/GridNode.h>
#include <lazykinoprm/State.h>

#define NONOPEN 99999

//TODO: remove this SearchNode Struct
//A star graph search node
// struct SearchNode;
// typedef SearchNode* SearchNodePtr;

// struct SearchNode
// {
//   Vector3d pose;
//   Vector3i index;
//   uint32_t listidx;
//   NodeState nodestate;
//   double pathCost,heurtCost,fnCost; // fn = pathCost + heurtCost;
//   void pushSearchNode(GridNodePtr _gridnode,NodeState _nodestate)
//   {
//     pose = _gridnode->pose;
//     index = _gridnode->index;
//     nodestate.CurrGridNodePtr = _gridnode;
//     nodestate = _nodestate;
//   }
// };

//A star graph search class
class OpenList
{
protected:
  // C++ https://en.cppreference.com/w/cpp/container/multimap for quick indexing
  //  but it's not suitable this condition, because GridNode is change in searching process
  //  their states is different
  //  GridNodePtr terminatePtr;
  //  std::multimap<double, GridNodePtr> openset;
  
  std::vector<Eigen::Vector3d> node_pose;  // node pose list
  std::vector<Eigen::Vector3i> node_index; // node index list
  std::vector<uint32_t> node_listidx;      // node index in openlist for for quick indexing
  // vector<Eigen::Vector3i> parent_index; //parent index list
  std::vector<uint32_t> parent_listidx; // parent node index in openlist for for quick indexing
  std::vector<double> pathCostSets;     // openlist fn score list

  // std::multimap<double, uint32_t>::iterator fnlist;
  // vector<AstarNode> nodesSets;  //node state list

  std::vector<double> fnCostSets; // openlist fn score list
  std::vector<uint32_t> fnlist;   // fnCostSets >> listidx

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
    fnlist.push_back(_listidx);
    fnCostSets.push_back(_fncost);
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
  uint32_t list_length;               // open list length
  std::vector<NodeState> nodestateSets; // node state list
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
        return _listidx;
      }
    }
    return _listidx;
  }

  bool IsAllExtend()
  {
    return fnCostSets.empty();
  }
  NodeState OpenListMinPop();
  // void insert(SearchNode _node_new, SearchNode _node_pare, NodeState _node_state);
  // void insert(SearchNodePtr _node_new, SearchNodePtr _node_pare, NodeState _node_state);
  void insert(NodeStatePtr _node_pare,NodeStatePtr _node_new);
  
  void reset();
// TODO
  OpenList()
  {
    list_length=0;
  }
  ~OpenList(){};
};

class CloseList
{
private:
                 // open list length
  std::vector<Eigen::Vector3d> node_pose;  // node pose list
  std::vector<Eigen::Vector3i> node_index; // node index list

public:
  uint32_t list_length;
//that function change the gridnode
  GridNodePtr insert(GridNodePtr _node)
  {
    _node->type = Extend;
    list_length++;
    node_pose.push_back(_node->pose);
    node_index.push_back(_node->index);
    return _node;
  }
  bool isCloseList(GridNodePtr _node)
  {
    bool flage = false;
    if (_node->type == Extend)
      flage = true;
    return flage;
  }
  void reset()
  {
    list_length = 0;
    node_pose.clear();
    node_index.clear();
  }
  CloseList(){
    list_length=0;
  }
  ~CloseList(){};
};
// //Lazy PRM class
// class LazyPRM
// {
//   private:
//   double c_angle; //angle cost weight
//   public:
//   double AngleMinDelta(Eigen::Vector3d _start,Eigen::Vector3d _goal);
//   double AngleCost(Eigen::Vector3d _start,Eigen::Vector3d _goal)
//   {
//     double angle,cost;
//     angle = AngleMinDelta(_start,_goal);
//     cost = c_angle * angle * angle ;
//   }
//   double getDeflection(Eigen::Vector3d _start,Eigen::Vector3d _goal)
//   {
//     double angle;
//     angle = atan2((_goal[2]-_start[2]),(_goal[1]-_start[1]));
//   }

// };

#endif //_LAZY_PRM_H