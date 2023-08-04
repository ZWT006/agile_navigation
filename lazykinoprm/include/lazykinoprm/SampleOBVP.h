/*
 * @Author: wentao zhang && zwt190315@163.com
 * @Date: 2023-04-03
 * @LastEditTime: 2023-05-04
 * @Description: 
 * @reference: 
 * 
 * Copyright (c) 2023 by wentao zhang, All Rights Reserved. 
 */




#ifndef _OBVP_H
#define _OBVP_H

#include <Eigen/Eigen>
#include <iostream>
#include <vector>
#include <math.h>

#include "Polynomial.h"
#include "GridNode.h"

#define xIDX 0
#define yIDX 1
#define qIDX 2

class SampleOBVP
{
private:
double vel_factor;
double ome_factor;
double costJ;
Eigen::Vector3d spi,svi,sai,spf,svf,saf;
double weightR;
double dt;
    /* data */
public:
    Polynomial poly;
    SampleOBVP(double _vel_factor,double _ome_factor,double _weightR,double _dt);
    ~SampleOBVP(){};
    // void SolveBVP(NodeStatePtr _istate,NodeStatePtr _fstate,PathStatePtr _pathstateptr);
    // void SolveMiniAccInputAcc(NodeStatePtr _istate,NodeStatePtr _fstate,PathStatePtr _pathstateptr);
    void SolveBVP(NodeStatePtr _istate,NodeStatePtr _fstate);
    void SolveMiniAccInputAcc(NodeStatePtr _istate,NodeStatePtr _fstate);
};

#endif //_OBVP_H