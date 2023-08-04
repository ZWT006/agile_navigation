/*
 * @Author: wentao zhang && zwt190315@163.com
 * @Date: 2023-07-05
 * @LastEditTime: 2023-07-11
 * @Description: read trajectory from csv file
 * @reference: 
 * 
 * Copyright (c) 2023 by wentao zhang, All Rights Reserved. 
 */

#ifndef READWRITECSV_HPP_
#define READWRITECSV_HPP_

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

class ReadCSV
{
    private:
    std::string _filename;
    std::ifstream _file;
    double _resolution = 1.0;
    double _biasx = 0.0;
    double _biasy = 0.0;

    public:
    ReadCSV() = default;
    
    ~ReadCSV(){};

    void setFileName(std::string filename){
        _filename = filename;
    }

    void setOrign(double res) {
        _resolution = res;
    }
    bool readFile(std::vector<double> &pxtraj, std::vector<double> &pytraj, std::vector<double> &pqtraj, 
                std::vector<double> &vxtraj, std::vector<double> &vytraj, std::vector<double> &vqtraj){
        if (!pxtraj.empty()){
            pxtraj.clear();pytraj.clear();pqtraj.clear();
            vxtraj.clear();vytraj.clear();vqtraj.clear();
        }
        _file.open(_filename,std::ios::in);
        if(!_file.is_open()){
            std::cout << "Error opening file" << std::endl;
            return false;
        }
        std::string lineStr;
        bool FIRSTLINE = true;
        // bool PRINT = true;
        while(std::getline(_file, lineStr)){
            std::string px, py, pq, vx, vy, vq;
            std::stringstream ss(lineStr);
            std::getline(ss, px, ',');
            std::getline(ss, py, ',');
            std::getline(ss, pq, ',');
            std::getline(ss, vx, ',');
            std::getline(ss, vy, ',');
            std::getline(ss, vq, ',');
            if (FIRSTLINE){
                FIRSTLINE = false;
                _biasx = -std::stod(px) * _resolution;
                _biasy = -std::stod(py) * _resolution;
                continue;
            }
            pxtraj.push_back(std::stod(px) * _resolution + _biasx);
            pytraj.push_back(std::stod(py) * _resolution + _biasy);
            pqtraj.push_back(std::stod(pq));
            vxtraj.push_back(std::stod(vx));
            vytraj.push_back(std::stod(vy));
            vqtraj.push_back(std::stod(vq));
            // if (PRINT){
            //     std::cout << "pos" << " px: " << pxtraj.at(0) << " py: " << pytraj.at(0) << " pq: " << pqtraj.at(0) << std::endl;
            //     std::cout << "vel" << " vx: " << vxtraj.at(0) << " vy: " << vytraj.at(0) << " vq: " << vqtraj.at(0) << std::endl;
            //     PRINT = false;
            // }
        }
        // ROS_INFO("read trajectory from csv file size: %ld",pxtraj.size());
        _file.close();
        return true;
    }
};


#endif // READWRITECSV_HPP_