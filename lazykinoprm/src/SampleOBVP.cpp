/*
 * @Author: wentao zhang && zwt190315@163.com
 * @Date: 2023-04-03
 * @LastEditTime: 2023-05-04
 * @Description: 
 * @reference: 
 * 
 * Copyright (c) 2023 by wentao zhang, All Rights Reserved. 
 */

#include <lazykinoprm/SampleOBVP.h>

#include <angles/angles.h>

using namespace std;
using namespace Eigen;

//return the min angle delta 
double AngleDelta(Eigen::Vector3d _start, Eigen::Vector3d _goal)
{
  double _anggoal, _angstart, _delta;
  _anggoal = _goal[2];
  _angstart = _start[2];
  // if (abs(_delta) > )
  _anggoal = fmod(_anggoal, M_PI * 2);
  if (_anggoal < 0)
    _anggoal = _anggoal + M_PI * 2;
  _angstart = fmod(_angstart, M_PI * 2);
  if (_angstart < 0)
    _angstart = _angstart + M_PI * 2;
  _delta = _anggoal - _angstart;
  if (abs(_delta) > M_PI)
  {
    if (_delta > 0)
      _delta = _delta - M_PI * 2;
    else
      _delta = _delta + M_PI * 2;
  }
  return _delta;
}

/**
 * @description: 
 * @reference: 
 * @param {double} _vel_factor
 * @param {double} _ome_factor
 * @param {double} _weightR
 * @param {double} _dt
 */
SampleOBVP::SampleOBVP(double _vel_factor, double _ome_factor, double _weightR, double _dt)
{
  vel_factor = _vel_factor;
  ome_factor = _ome_factor;
  weightR = _weightR;
  dt = _dt;
}

// void SampleOBVP::SolveBVP(NodeStatePtr _istate, NodeStatePtr _fstate, PathStatePtr _pathstateptr)
/**
 * @description: solve obvp fix start;final fix time, also update the _fstate states
 * @reference: 
 * @param {NodeStatePtr} _istate
 * @param {NodeStatePtr} _fstate
 * @return 
 */
void SampleOBVP::SolveBVP(NodeStatePtr _istate, NodeStatePtr _fstate)
{
  double deltaq = 0;
  double T = 999.9;
  Vector3d si, sf;
  vector<double> coeff(6, 0);
  vector<double> xcoeff(6, 0), ycoeff(6, 0), qcoeff(6, 0);
  spi = _istate->Position;
  svi = _istate->Velocity;
  sai = _istate->Acceleration;
  spf = _fstate->Position;
  svf = _fstate->Velocity;
  saf = _fstate->Acceleration;
  // _pathstateptr->spf = spf;
  // _pathstateptr->spi = spi;
  // _pathstateptr->svi = svi;
  // _pathstateptr->sai = sai;
  // _pathstateptr->svf = svf;
  // _pathstateptr->saf = saf;
  // #####################################
  // Compute coefficients of quintic polynomial for joint position
  // Compute coefficients of quintic polynomial for joint velocity
  // Compute coefficients of quintic polynomial for joint acceleration
  // Compute coefficients of quintic polynomial for joint jerk
  // Compute coefficients of quintic polynomial for joint snap
  // Compute coefficients of quintic polynomial for joint crac
  // angle modify
  // deltaq = AngleDelta(spi, spf);
  deltaq = angles::shortest_angular_distance(spi[qIDX], spf[qIDX]);
  spf[qIDX] = spi[qIDX] + deltaq;
  // step2: 求参考时间,线速度时间和角速度时间取较大值
  T = max(sqrt(pow(spi[xIDX] - spf[xIDX], 2) + pow(spi[yIDX] - spf[yIDX], 2)) / vel_factor, abs(deltaq) / ome_factor);
  // step3: obvp 计算
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  si[0] = spi[xIDX];
  si[1] = svi[xIDX], si[2] = sai[xIDX];
  sf[0] = spf[xIDX];
  sf[1] = svf[xIDX], sf[2] = saf[xIDX];
  coeff[0] = si[0];
  coeff[1] = si[1];
  coeff[2] = si[2] / 2;
  coeff[3] = -((-20 * sf[0] + 20 * si[0] - sf[2] * pow(T, 2) + 3 * si[2] * pow(T, 2) + 8 * T * sf[1] + 12 * T * si[1]) / (2 * pow(T, 3)));
  coeff[4] = -((30 * sf[0] - 30 * si[0] + 2 * sf[2] * pow(T, 2) - 3 * si[2] * pow(T, 2) - 14 * T * sf[1] - 16 * T * si[1]) / (2 * pow(T, 4)));
  coeff[5] = -((-12 * sf[0] + 12 * si[0] - sf[2] * pow(T, 2) + si[2] * pow(T, 2) + 6 * T * sf[1] + 6 * T * si[1]) / (2 * pow(T, 5)));
  // xcoeff.assign(coeff.end(), coeff.begin());
  std::reverse_copy(coeff.begin(), coeff.end(),xcoeff.begin());
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  si[0] = spi[yIDX];
  si[1] = svi[yIDX], si[2] = sai[yIDX];
  sf[0] = spf[yIDX];
  sf[1] = svf[yIDX], sf[2] = saf[yIDX];
  coeff[0] = si[0];
  coeff[1] = si[1];
  coeff[2] = si[2] / 2;
  coeff[3] = -((-20 * sf[0] + 20 * si[0] - sf[2] * pow(T, 2) + 3 * si[2] * pow(T, 2) + 8 * T * sf[1] + 12 * T * si[1]) / (2 * pow(T, 3)));
  coeff[4] = -((30 * sf[0] - 30 * si[0] + 2 * sf[2] * pow(T, 2) - 3 * si[2] * pow(T, 2) - 14 * T * sf[1] - 16 * T * si[1]) / (2 * pow(T, 4)));
  coeff[5] = -((-12 * sf[0] + 12 * si[0] - sf[2] * pow(T, 2) + si[2] * pow(T, 2) + 6 * T * sf[1] + 6 * T * si[1]) / (2 * pow(T, 5)));
  // ycoeff.assign(coeff.end(), coeff.begin());
  std::reverse_copy(coeff.begin(), coeff.end(),ycoeff.begin());
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  si[0] = spi[qIDX];
  si[1] = svi[qIDX], si[2] = sai[qIDX];
  sf[0] = spf[qIDX];
  sf[1] = svf[qIDX], sf[2] = saf[qIDX];
  coeff[0] = si[0];
  coeff[1] = si[1];
  coeff[2] = si[2] / 2;
  coeff[3] = -((-20 * sf[0] + 20 * si[0] - sf[2] * pow(T, 2) + 3 * si[2] * pow(T, 2) + 8 * T * sf[1] + 12 * T * si[1]) / (2 * pow(T, 3)));
  coeff[4] = -((30 * sf[0] - 30 * si[0] + 2 * sf[2] * pow(T, 2) - 3 * si[2] * pow(T, 2) - 14 * T * sf[1] - 16 * T * si[1]) / (2 * pow(T, 4)));
  coeff[5] = -((-12 * sf[0] + 12 * si[0] - sf[2] * pow(T, 2) + si[2] * pow(T, 2) + 6 * T * sf[1] + 6 * T * si[1]) / (2 * pow(T, 5)));
  // qcoeff.assign(coeff.end(), coeff.begin());
  std::reverse_copy(coeff.begin(), coeff.end(),qcoeff.begin());
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // step4: 计算cost
  // TODO: cost calculate
  _fstate->referT = T;
  _fstate->pushcoeff(xcoeff, ycoeff, qcoeff);
  _fstate->ParePosition = spi;
  _fstate->PareVelocity = svi;
  _fstate->PareAcceleration = sai;
}

// void SampleOBVP::SolveMiniAccInputAcc(NodeStatePtr _istate,NodeStatePtr _fstate,PathStatePtr _pathstateptr)
/**
 * @description: solve fix start; free end; reference T,also update the _fstate states
 * @reference: 
 * @param {NodeStatePtr} _istate
 * @param {NodeStatePtr} _fstate
 * @param {PathStatePtr} _pathstateptr
 * @return {*}
 */
void SampleOBVP::SolveMiniAccInputAcc(NodeStatePtr _istate,NodeStatePtr _fstate)
{
  // step1: 计算轨迹点的多项式系数;
  double deltaq = 0;
  double T = 999.9;
  double alphax = 0, alphay = 0, alphaq = 0;
  Vector3d si, sf;
  vector<double> coeff(4, 0);
  vector<double> xcoeff(4, 0), ycoeff(4, 0), qcoeff(4, 0);
  spi = _istate->Position;
  svi = _istate->Velocity;
  sai = _istate->Acceleration;
  spf = _fstate->Position;
  deltaq = AngleDelta(spi, spf);
  spf[qIDX] = spi[qIDX] + deltaq;
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // step2: 求参考时间,线速度时间和角速度时间取较大值
  T = max(sqrt(pow(spi[xIDX] - spf[xIDX], 2) + pow(spi[yIDX] - spf[yIDX], 2)) / vel_factor, abs(deltaq) / ome_factor);
  // step3: obvp 计算
  alphax = 3 * (svi[xIDX] * T + spi[xIDX] - spf[xIDX]) / pow(T, 3);
  alphay = 3 * (svi[yIDX] * T + spi[yIDX] - spf[yIDX]) / pow(T, 3);
  alphaq = 3 * (svi[qIDX] * T + spi[qIDX] - spf[qIDX]) / pow(T, 3) * weightR;         
  //# Tag 对这里的*obj.R抱有疑问,不确定是不是会影响结果准确性
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  coeff[0] = spi[xIDX];
  coeff[1] = svi[xIDX];
  coeff[2] = -alphax/2*T;
  coeff[3] = alphax/6;
  svf[0] = coeff[3]*3*pow(T,2)+coeff[2]*2*T+coeff[1];
  saf[0] = coeff[3]*6*T+coeff[2]*2;
  // xcoeff.assign(coeff.end(), coeff.begin());
  std::reverse_copy(coeff.begin(), coeff.end(), xcoeff.begin());

  // for code Debug 
  // cout << BLUE << "[SampleOBVP]" << RESET << "coeff: " << coeff[0] << " " << coeff[1] << " " << coeff[2] << " " << coeff[3] << endl;
  // cout << BLUE << "[SampleOBVP]" << RESET << "xcoeff: " << xcoeff[0] << " " << xcoeff[1] << " " << xcoeff[2] << " " << xcoeff[3] << endl;
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  coeff[0] = spi[yIDX];
  coeff[1] = svi[yIDX];
  coeff[2] = -alphay/2*T;
  coeff[3] = alphay/6;
  svf[1] = coeff[3]*3*pow(T,2)+coeff[2]*2*T+coeff[1];
  saf[1] = coeff[3]*6*T+coeff[2]*2;
  // ycoeff.assign(coeff.end(), coeff.begin());
  std::reverse_copy(coeff.begin(), coeff.end(), ycoeff.begin());
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  coeff[0] = spi[qIDX];
  coeff[1] = svi[qIDX];
  coeff[2] = -alphaq/2*T;
  coeff[3] = alphaq/6;
  svf[2] = coeff[3]*3*pow(T,2)+coeff[2]*2*T+coeff[1];
  saf[2] = coeff[3]*6*T+coeff[2]*2;
  // qcoeff.assign(coeff.end(), coeff.begin());
  std::reverse_copy(coeff.begin(), coeff.end(), qcoeff.begin());
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // step4: 计算cost
  //set fsate node state
  _fstate->referT = T;
  _fstate->trajectory_cost = alphax/3*pow(T,3)+alphay/3*pow(T,3)+alphaq/3*pow(T,3)*weightR;
  _fstate->pushcoeff(xcoeff, ycoeff, qcoeff);
  _fstate->Velocity = svf;
  _fstate->Acceleration = saf;
  
  _fstate->ParePosition = spi;
  _fstate->PareVelocity = svi;
  _fstate->PareAcceleration = sai;
}