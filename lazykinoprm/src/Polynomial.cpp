/*
 * @Author: wentao zhang && zwt190315@163.com
 * @Date: 2023-12-21
 * @LastEditTime: 2023-12-21
 * @Description: 
 * @reference: 
 * 
 * Copyright (c) 2023 by wentao zhang, All Rights Reserved. 
 */
/*
 * @Author: wentao zhang && zwt190315@163.com
 * @Date: 2023-04-03
 * @LastEditTime: 2023-05-04
 * @Description: 
 * @reference: 
 * 
 * Copyright (c) 2023 by wentao zhang, All Rights Reserved. 
 */

#include <lazykinoprm/Polynomial.h>

/**
 * @description: MATLAB polyval function
 * @reference: 
 * @param {vector<double>} coeffs
 * @param {vector<double>} values
 * @return {vector<double>} results
 */
std::vector<double> Polynomial::PolyvalVector(std::vector<double> coeffs, std::vector<double> values)
{
  std::vector<double> results;
  for (auto const &val : values)
  {
    double result = 0;
    for (int i = 0, deg = coeffs.size() - 1; deg >= 0; deg--, i++)
    {
      result += coeffs[i] * std::pow(val, deg);
    }
    results.push_back(result);
  }
  return results;
}

/**
 * @description: 
 * @reference: 
 * @param {vector<double>} coeffs: 
 * @param {double} value
 * @return {double} result
 */
double Polynomial::PolyvalDot(std::vector<double> coeffs, double value)
{
  double result = 0;
  for (int i = 0, deg = coeffs.size() - 1; deg >= 0; deg--, i++)
  {
    result += coeffs[i] * std::pow(value, deg);
  }
  return result;
}

/**
 * @description: MATLAB polyder function
 * @reference: 
 * @param {vector<double>} coeffs
 * @return {vector<double>}
 */
std::vector<double> Polynomial::Polyder(std::vector<double> coeffs)
{
  std::vector<double> dcoeffs;
  double result = 0;
    for (int i = 0, deg = coeffs.size() - 1; deg > 0; deg--, i++)
    {
      result = coeffs[i]*deg;
      dcoeffs.push_back(result);
    }
    return dcoeffs;  
}

/**
 * @description: get sequence from ds to dg by internal dt
 * @reference: 
 * @param {double} ds
 * @param {double} dg
 * @param {double} dt
 * @return {vector<double>} seque
 */
std::vector<double> Polynomial::getseqence(double ds, double dg, double dt)
{
  std::vector<double> seque;
  for (double _seq = ds; _seq < dg; _seq = _seq + dt)
    seque.push_back(_seq);
  return seque;
}