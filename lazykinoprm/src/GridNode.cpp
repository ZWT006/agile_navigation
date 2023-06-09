/*
 * @Author: wentao zhang && zwt190315@163.com
 * @Date: 2023-04-03
 * @LastEditTime: 2023-05-04
 * @Description: 
 * @reference: 
 * 
 * Copyright (c) 2023 by wentao zhang, All Rights Reserved. 
 */



#include "lazykinoprm/GridNode.h"

// GridNode::GridNode(Eigen::Vector3d _Pose, Eigen::Vector3i _Index, GridNodeType_Set _Type)
// {
//   type = _Type;
//   pose = _Pose;
//   index = _Index;
// }

// set Pose function
void GridNode::setPose(Eigen::Vector3d _Pose, Eigen::Vector3i _Index)
{
  pose = _Pose;
  index = _Index;
}
// set Type function
void GridNode::setType(GridNodeType_Set _Type)
{
  type = _Type;
}

// ############################################################################
//
// node state structure
void NodeState::setCollisioncheck(bool _collision_check)
{
  collision_check = _collision_check;
}
void NodeState::setSelfNodeState(Eigen::Vector3d _Position, Eigen::Vector3d _Velocity, Eigen::Vector3d _Acceleration)
{
  Position = _Position;
  Velocity = _Velocity;
  Acceleration = _Acceleration;
}
void NodeState::setParentNodeState(Eigen::Vector3d _Position, Eigen::Vector3d _Velocity, Eigen::Vector3d _Acceleration)
{
  ParePosition = _Position;
  PareVelocity = _Velocity;
  PareAcceleration = _Acceleration;
}

/**********************************************************************************************************
 * @description: get the polynomial trajectory
 * @reference: 
 * @param {char} _type
 * @param {char} _dimen
 * @param {double} _dt
 * @return {*}
 */
std::vector<double> NodeState::polytraj(char _type, char _dimen, double _dt)
{
  std::vector<double> traj;
  std::vector<double> timeseq;
  timeseq = poly.getseqence(0.0, referT, _dt);
  if (_type == 'p' && _dimen == 'x')
    traj = poly.PolyvalVector(xpcoeff, timeseq);
  else if (_type == 'p' && _dimen == 'y')
    traj = poly.PolyvalVector(ypcoeff, timeseq);
  else if (_type == 'p' && _dimen == 'q')
    traj = poly.PolyvalVector(qpcoeff, timeseq);
  else if (_type == 'v' && _dimen == 'x')
    traj = poly.PolyvalVector(xvcoeff, timeseq);
  else if (_type == 'v' && _dimen == 'y')
    traj = poly.PolyvalVector(yvcoeff, timeseq);
  else if (_type == 'v' && _dimen == 'q')
    traj = poly.PolyvalVector(qvcoeff, timeseq);
  else if (_type == 'a' && _dimen == 'x')
    traj = poly.PolyvalVector(xacoeff, timeseq);
  else if (_type == 'a' && _dimen == 'y')
    traj = poly.PolyvalVector(yacoeff, timeseq);
  else if (_type == 'a' && _dimen == 'q')
    traj = poly.PolyvalVector(qacoeff, timeseq);
  else
  {
    std::cout << RED << "Error: wrong type or dimen! for NodeState polytraj" << RESET << std::endl;
  }
  return traj;
}

double NodeState::polytrajStart(char _type, char _dimen)
{
  double point; 
  if (_type == 'p' && _dimen == 'x')
    point = poly.PolyvalDot(xpcoeff, 0.0);
  else if (_type == 'p' && _dimen == 'y')
    point = poly.PolyvalDot(ypcoeff, 0.0);
  else if (_type == 'p' && _dimen == 'q')
    point = poly.PolyvalDot(qpcoeff, 0.0);
  else if (_type == 'v' && _dimen == 'x')
    point = poly.PolyvalDot(xvcoeff, 0.0);
  else if (_type == 'v' && _dimen == 'y')
    point = poly.PolyvalDot(yvcoeff, 0.0);
  else if (_type == 'v' && _dimen == 'q')
    point = poly.PolyvalDot(qvcoeff, 0.0);
  else if (_type == 'a' && _dimen == 'x')
    point = poly.PolyvalDot(xacoeff, 0.0);
  else if (_type == 'a' && _dimen == 'y')
    point = poly.PolyvalDot(yacoeff, 0.0);
  else if (_type == 'a' && _dimen == 'q')
    point = poly.PolyvalDot(qacoeff, 0.0);
  else
  {
    std::cout << RED << "Error: wrong type or dimen! for NodeState polytrajStart" << RESET << std::endl;
  }
}

double NodeState::polytrajEnd(char _type, char _dimen)
{
  double point;
  if (_type == 'p' && _dimen == 'x')
    point = poly.PolyvalDot(xpcoeff, referT);
  else if (_type == 'p' && _dimen == 'y')
    point = poly.PolyvalDot(ypcoeff, referT);
  else if (_type == 'p' && _dimen == 'q')
    point = poly.PolyvalDot(qpcoeff, referT);
  else if (_type == 'v' && _dimen == 'x')
    point = poly.PolyvalDot(xvcoeff, referT);
  else if (_type == 'v' && _dimen == 'y')
    point = poly.PolyvalDot(yvcoeff, referT);
  else if (_type == 'v' && _dimen == 'q')
    point = poly.PolyvalDot(qvcoeff, referT);
  else if (_type == 'a' && _dimen == 'x')
    point = poly.PolyvalDot(xacoeff, referT);
  else if (_type == 'a' && _dimen == 'y')
    point = poly.PolyvalDot(yacoeff, referT);
  else if (_type == 'a' && _dimen == 'q')
    point = poly.PolyvalDot(qacoeff, referT);
  else
  {
    std::cout << RED << "Error: wrong type or dimen! for NodeState polytrajEnd" << RESET << std::endl;
  }
}

/**********************************************************************************************************
  * @description: push the coeff of the polynomial trajectory 
  * @reference: 
  * @param {std::vector<double>} _xcoeff
  * @param {std::vector<double>} _ycoeff
  * @param {std::vector<double>} _qcoeff
  * @return {void}
*/
void NodeState::pushcoeff(std::vector<double> _xcoeff,std::vector<double> _ycoeff,std::vector<double> _qcoeff)
{
  xpcoeff = _xcoeff;
  ypcoeff = _ycoeff;
  qpcoeff = _qcoeff;
  xvcoeff = poly.Polyder(xpcoeff);
  yvcoeff = poly.Polyder(ypcoeff);
  qvcoeff = poly.Polyder(qpcoeff);
  xacoeff = poly.Polyder(xvcoeff);
  yacoeff = poly.Polyder(yvcoeff);
  qacoeff = poly.Polyder(qvcoeff);
}


/**
 * @description: 
 * @reference: 
 * @param {GridNodePtr} _CurrGridNodePtr
 * @return {*}
 */
void NodeState::pushGridNode(GridNodePtr _CurrGridNodePtr)
{
  CurrGridNodePtr = _CurrGridNodePtr;  
}

void NodeState::clear()
{
  Position = Eigen::Vector3d::Zero();
  Velocity = Eigen::Vector3d::Zero();
  Acceleration = Eigen::Vector3d::Zero();
  CurrGridNodePtr = nullptr;
  Currnodelistindex = 0;
  ParePosition = Eigen::Vector3d::Zero();
  PareVelocity = Eigen::Vector3d::Zero();
  PareAcceleration = Eigen::Vector3d::Zero();
  PareGridNodePtr = nullptr;
  Parenodelistindex = 0;
  xpcoeff.clear();
  ypcoeff.clear();
  qpcoeff.clear();
  xvcoeff.clear();
  yvcoeff.clear();
  qvcoeff.clear();
  xacoeff.clear();
  yacoeff.clear();
  qacoeff.clear();
  collision_check = false;
}
// ############################################################################
//
// path state structure
/**
 * @description: 
 * @reference: 
 * @param {char} _type = 'p' -> position; 'v' -> velocity; 'a' -> acceleration
 * @param {char} _dimen = 'x' -> x; 'y' -> y; 'q' -> q
 * @param {double} _dt = time interval
 * @return {vector<double>}  
 */
std::vector<double> PathState::polytraj(char _type,char _dimen,double _dt)
{
  std::vector<double> traj;
  std::vector<double> timeseq;
  timeseq = poly.getseqence(0.0, referT, _dt);
  if(_type == 'p' && _dimen == 'x')
  traj = poly.PolyvalVector(xpcoeff, timeseq);
  else if(_type == 'p' && _dimen == 'y')
  traj = poly.PolyvalVector(ypcoeff, timeseq);
  else if(_type == 'p' && _dimen == 'q')
  traj = poly.PolyvalVector(qpcoeff, timeseq);
  else if(_type == 'v' && _dimen == 'x')
  traj = poly.PolyvalVector(xvcoeff, timeseq);
  else if(_type == 'v' && _dimen == 'y')
  traj = poly.PolyvalVector(yvcoeff, timeseq);
  else if(_type == 'v' && _dimen == 'q')
  traj = poly.PolyvalVector(qvcoeff, timeseq);
  else if(_type == 'a' && _dimen == 'x')
  traj = poly.PolyvalVector(xacoeff, timeseq);
  else if(_type == 'a' && _dimen == 'y')
  traj = poly.PolyvalVector(yacoeff, timeseq);
  else if(_type == 'a' && _dimen == 'q')
  traj = poly.PolyvalVector(qacoeff, timeseq);
  else
  {
    std::cout << "Error: wrong type or dimen!" << std::endl;
  }
  return traj;
}

// std::vector<double> xtraj(){};
// std::vector<double> ytraj(){};
// std::vector<double> qtraj(){};
// std::vector<double> PathState::xtraj()
// {
//   std::vector<double> traj;
//   std::vector<double> timeseq;
//   timeseq = poly.getseqence(0.0, referT, dt);
//   traj = poly.PolyvalVector(xcoeff, timeseq);
//   return traj;
// }
// std::vector<double> PathState::ytraj()
// {
//   std::vector<double> traj;
//   std::vector<double> timeseq;
//   timeseq = poly.getseqence(0.0, referT, dt);
//   traj = poly.PolyvalVector(ycoeff, timeseq);
//   return traj;
// }
// std::vector<double> PathState::qtraj(double dt)
// {
//   std::vector<double> traj;
//   std::vector<double> timeseq;
//   timeseq = poly.getseqence(0.0, referT, dt);
//   traj = poly.PolyvalVector(ycoeff, timeseq);
//   return traj;
// }