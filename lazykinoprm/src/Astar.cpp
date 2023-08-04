/*
 * @Author: wentao zhang && zwt190315@163.com
 * @Date: 2023-04-03
 * @LastEditTime: 2023-05-04
 * @Description: 
 * @reference: 
 * 
 * Copyright (c) 2023 by wentao zhang, All Rights Reserved. 
 */

#include <lazykinoprm/Astar.h>
#include <sstream>

using namespace std;
using namespace Eigen;

// pop the min fncost from openlist
AstarNode OpenList::OpenListMinPop()
{
  if (IsAllExtend())
  {
    ROS_WARN("openlist is empty");
  }
  uint32_t min_idx;
  AstarNode _node;
  min_idx=fnlist.front();
  _node.pose = node_pose[min_idx];
  _node.index = node_index[min_idx];
  _node.listidx = min_idx;
  _node.pathCost = pathCostSets[min_idx];
  _node.fnCost = fnCostSets[min_idx];
  _node.state  = nodestateSets[min_idx];
  EraseFnlist(min_idx);
}

//insert openlist node 
void OpenList::insert(AstarNodePtr _node_new,AstarNodePtr _node_pare,NodeState _node_state)
// I don't the difference between Ptr and normal in this place 
// void OpenList::insert(AstarNode _node_new,AstarNode _node_pare,NodeState _node_state)
{
  uint32_t new_listidx = isOpenList(_node_new->index);
  double _fnCost = _node_new->pathCost + _node_new->heurtCost;
  // listidxx = 0 >> startnode but it have been extend at begin
  //so curr_listidx == NONOPEN means _node_new isn't inopenlist
  if (new_listidx == NONOPEN)
  {
    node_pose.push_back(_node_new->pose);
    node_index.push_back(_node_new->index);
    node_listidx.push_back(list_length);
    nodestateSets.push_back(_node_state);
    pathCostSets.push_back(_node_new->pathCost);
    parent_listidx.push_back(_node_pare->listidx);
    InsertFnlist(_node_new->fnCost,_node_new->listidx);
    list_length++;
  }
  else
  {
    if(fnCostSets[new_listidx] > _node_new->fnCost)
    {
      EraseFnlist(new_listidx);
      InsertFnlist(_node_new->fnCost, _node_new->listidx);
      node_pose[new_listidx] = _node_new->pose;
      node_index[new_listidx] = _node_new->index;
      node_listidx[new_listidx] = list_length;
      nodestateSets[new_listidx] = _node_state;
      pathCostSets[new_listidx] = _node_new->pathCost;
      parent_listidx[new_listidx] = _node_pare->listidx;
    }
  }
}
