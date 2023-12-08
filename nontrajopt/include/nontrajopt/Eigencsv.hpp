/*
 * @Author: wentao zhang && zwt190315@163.com
 * @Date: 2023-07-12
 * @LastEditTime: 2023-11-27
 * @Description: Eigen 的 Matrix/Vector 与 csv 文件的转换
 * @reference: 
 * 
 * Copyright (c) 2023 by wentao zhang, All Rights Reserved. 
 */

#ifndef EIGENCSV_HPP_
#define EIGENCSV_HPP_

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Eigen>

class EigenCSV {

private:
std::string titles;

public:
    EigenCSV() = default;
    ~EigenCSV() = default;
    bool FirstLineIsTitles = false;
    void setTitles(const std::string& _titles) {
        titles = _titles;
    }

    bool WriteVector(const Eigen::VectorXd& vec, const std::string& _filename) {
        std::ofstream file(_filename);
        if (file) {
            file.close();
            file.open(_filename, std::ios_base::out | std::ios_base::trunc);
        }
        // if (!file.is_open()) {
        //     std::cout << "Open file failed!" << std::endl;
        //     return false;
        // }
        if (FirstLineIsTitles) {
            file << titles << std::endl;
        }
        for (int i = 0; i < vec.size(); i++) {
            file << vec(i) << std::endl;
        }
        file.close();
        return true;
    }
    bool WriteMatrix(const Eigen::MatrixXd& mat, const std::string& _filename) {
        std::ofstream file(_filename);
        if (file) {
            file.close();
            file.open(_filename, std::ios_base::out | std::ios_base::trunc);
        }
        // if (!file.is_open()) {
        //     std::cout << "Open file failed!" << std::endl;
        //     return false;
        // }
        if (FirstLineIsTitles) {
            file << titles << std::endl;
        }
        for (int i = 0; i < mat.rows(); i++) {
            for (int j = 0; j < mat.cols(); j++) {
                file << mat(i, j) << ",";
            }
            file << std::endl;
        }
        file.close();
        return true;
    }
    bool ReadVector(Eigen::VectorXd& vec, const std::string& _filename) {
        std::ifstream file(_filename);
        if (!file.is_open()) {
            std::cout << "Open file failed!" << std::endl;
            return false;
        }
        std::string line;
        if (FirstLineIsTitles) { // 读取标题行 但是 跳过
            std::getline(file, line);
        }
        std::vector<double> vec_temp;
        while (std::getline(file, line)) {
            std::stringstream ss(line);
            std::string str;
            while (std::getline(ss, str, ',')) {
                vec_temp.push_back(std::stod(str));
            }
        }
        vec.resize(vec_temp.size());
        for (int i = 0; i < vec_temp.size(); i++) {
            vec(i) = vec_temp[i];
        }
        file.close();
        return true;
    }
    bool ReadMatrix(Eigen::MatrixXd& mat, const std::string& _filename) {
        std::cout << _filename << std::endl;
        std::ifstream file(_filename);
        if (!file.is_open()) {
            std::cout << "Open file failed!" << std::endl;
            return false;
        }
        std::string line;
        if (FirstLineIsTitles) { // 读取标题行 但是 跳过
            std::getline(file, line);
        }
        std::vector<double> mat_temp;
        bool first_line = true;
        int cols = 0;
        while (std::getline(file, line)) {
            std::stringstream ss(line);
            std::string str;
            while (std::getline(ss, str, ',')) {
                mat_temp.push_back(std::stod(str));
                if (first_line) {
                    cols++;
                }
            }
            first_line = false;
        }
        mat.resize(mat_temp.size() / cols, cols);
        for (int i = 0; i < mat_temp.size() / cols; i++) {
            for (int j = 0; j < cols; j++) {
                mat(i, j) = mat_temp[i * cols + j];
            }
        }
        file.close();
        return true;
    }
};


#endif // EIGENCSV_HPP_