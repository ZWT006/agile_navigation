/*
 * @Author: wentao zhang && zwt190315@163.com
 * @Date: 2023-06-16
 * @LastEditTime: 2023-06-16
 * @Description: 
 * @reference: 
 * 
 * Copyright (c) 2023 by wentao zhang, All Rights Reserved. 
 */

#ifndef _SEGMENT_H_
#define _SEGMENT_H_

#include <vector>
#include <iostream>
#include <Eigen/Eigen>

struct Segment;
typedef Segment* SegmentPtr;

struct Segment
{
    double duration;
    std::vector<double> pxcoeff;
    std::vector<double> pycoeff;
    std::vector<double> pqcoeff;
};



#endif