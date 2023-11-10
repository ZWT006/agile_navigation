/*
 * @Author: wentao zhang && zwt190315@163.com
 * @Date: 2023-04-03
 * @LastEditTime: 2023-11-08
 * @Description: 
 * @reference: 
 * 
 * Copyright (c) 2023 by wentao zhang, All Rights Reserved. 
 */

#include <lazykinoprm/LazyPRM.h>
#include <sstream>
#include <cstdlib>

using namespace std;
using namespace Eigen;

// pop the min fncost from openlist
NodeState OpenList::OpenListMinPop()
{
  if (IsAllExtend())
  {
    ROS_WARN("openlist is empty");
  }
  uint32_t min_idx;
  NodeState _node;
  min_idx=fnlist.front();
  // _node.pose = node_pose[min_idx];
  // _node.index = node_index[min_idx];
  // _node.listidx = min_idx;
  // _node.pathCost = pathCostSets[min_idx];
  // _node.fnCost = fnCostSets[min_idx];
  _node  = nodestateSets[min_idx];
  EraseFnlist(min_idx);
  return _node;
}

//# Tag:  I don't know the difference between Ptr and normal in this place 
//insert openlist node 
void OpenList::insert(NodeStatePtr _node_pare,NodeStatePtr _node_new)
{
  uint32_t new_listidx = isOpenList(_node_new->CurrGridNodePtr->index);
  double _fnCost = _node_new->pathcost + _node_new->heurcost;
  _node_new->fncost = _fnCost;
  // listidxx = 0 >> startnode but it have been extend at begin
  //so curr_listidx == NONOPEN means _node_new isn't inopenlist
  if (new_listidx == NONOPEN)
  {
    /*###################################################*/
    // update the new node state
    _node_new->Currnodelistindex = list_length;
    _node_new->PareGridNodePtr = _node_pare->CurrGridNodePtr;
    _node_new->Parenodelistindex = _node_pare->Currnodelistindex;
    // _node_new->ParePosition = _node_pare->Position;
    // _node_new->PareVelocity = _node_pare->Velocity;
    // _node_new->PareAcceleration = _node_pare->Acceleration;
    /*###################################################*/
    // update the openlist
    node_pose.push_back(_node_new->CurrGridNodePtr->pose);
    node_index.push_back(_node_new->CurrGridNodePtr->index);
    node_listidx.push_back(list_length);
    // nodestateSets.push_back(_node_state);
    nodestateSets.push_back(*_node_new);
    pathCostSets.push_back(_node_new->pathcost);
    parent_listidx.push_back(_node_pare->Currnodelistindex);
    InsertFnlist(_node_new->fncost,list_length);
    list_length++;
    // ROS_INFO("insert new node");
  }
  else
  {
    if(fnCostSets[new_listidx] > _node_new->fncost)
    {
      /*###################################################*/
      // update the new node state
      _node_new->Currnodelistindex = new_listidx;
      _node_new->PareGridNodePtr = _node_pare->CurrGridNodePtr;
      _node_new->Parenodelistindex = _node_pare->Currnodelistindex;
      // _node_new->ParePosition = _node_pare->Position;
      // _node_new->PareVelocity = _node_pare->Velocity;
      // _node_new->PareAcceleration = _node_pare->Acceleration;
      /*###################################################*/
      // update the openlist
      EraseFnlist(new_listidx);
      InsertFnlist(_node_new->fncost, new_listidx);
      node_pose[new_listidx] = _node_new->CurrGridNodePtr->pose;
      node_index[new_listidx] = _node_new->CurrGridNodePtr->index;
      node_listidx[new_listidx] = new_listidx;
      // nodestateSets[new_listidx] = _node_state;
      nodestateSets[new_listidx] = *_node_new;
      pathCostSets[new_listidx] = _node_new->pathcost;
      parent_listidx[new_listidx] = _node_pare->Currnodelistindex;
      // ROS_INFO("update node");
    }
  }
}

//reset openlist
void OpenList::reset()
{
  node_pose.clear();
  node_index.clear();
  node_listidx.clear();
  nodestateSets.clear();
  pathCostSets.clear();
  parent_listidx.clear();
  fnlist.clear();
  fnCostSets.clear();
  list_length = 0;
}

//Debug
void OpenList::printnodelistidx()
{
    for (int idx = 0; idx < list_length; idx++)
    {
      cout << "node_listidx[" << idx << "]="<< node_listidx[idx] << endl;
    }
}
// //return the min angle delta 
// double LazyPRM::AngleMinDelta(Eigen::Vector3d _start, Eigen::Vector3d _goal)
// {
//   double _anggoal, _angstart, _delta;
//   _anggoal = _goal[3];
//   _angstart = _start[3];
//   // if (abs(_delta) > )
//   _anggoal = fmod(_anggoal, M_PI * 2);
//   if (_anggoal < 0)
//     _anggoal = _anggoal + M_PI * 2;
//   _angstart = fmod(_angstart, M_PI * 2);
//   if (_angstart < 0)
//     _angstart = _angstart + M_PI * 2;
//   _delta = _anggoal - _angstart;
//   if (abs(_delta) > M_PI)
//   {
//     if (_delta > 0)
//       _delta = _delta - M_PI * 2;
//     else
//       _delta = _delta + M_PI * 2;
//   }
// }
