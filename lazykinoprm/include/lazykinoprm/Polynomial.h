/*
 * @Author: wentao zhang && zwt190315@163.com
 * @Date: 2023-04-03
 * @LastEditTime: 2023-04-05
 * @Description: 
 * @reference: 
 * 
 * Copyright (c) 2023 by wentao zhang, All Rights Reserved. 
 */




#ifndef _POLYNOMIAL_H
#define _POLYNOMIAL_H

#include<iostream>
#include<vector>
#include<math.h>

class Polynomial
{
private:
    /* data */
public:
    Polynomial(/* args */){};
    ~Polynomial(){};
    std::vector<double> PolyvalVector(std::vector<double> coeffs, std::vector<double> values);

    double PolyvalDot(std::vector<double> coeffs, double values);

    std::vector<double> Polyder(std::vector<double> coeffs);

    std::vector<double> getseqence(double ds, double dg, double dt);
};

#endif //_POLYNOMIAL_H