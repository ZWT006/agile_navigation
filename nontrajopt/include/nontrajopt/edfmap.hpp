/*
 * @Author: wentao zhang && zwt190315@163.com
 * @Date: 2023-07-05
 * @LastEditTime: 2023-07-25
 * @Description: Euclidean Distance Field Map
 * @reference: 
 * 
 * Copyright (c) 2023 by wentao zhang, All Rights Reserved. 
 */

#ifndef EDFMAP_HPP_
#define EDFMAP_HPP_

#include <cmath>
#include <Eigen/Eigen>

#include <opencv2/opencv.hpp>

#define DOUBLE_ZERO 1e-6

#define RESET   "\033[0m"       /* Reset */
#define BLACK   "\033[30m"      /* Black */
#define RED     "\033[31m"      /* Red */
#define GREEN   "\033[32m"      /* Green */
#define YELLOW  "\033[33m"      /* Yellow */
#define BLUE    "\033[34m"      /* Blue */
#define MAGENTA "\033[35m"      /* Magenta */
#define CYAN    "\033[36m"      /* Cyan */
#define WHITE   "\033[37m"      /* White */
#define BOLDBLACK   "\033[1m\033[30m"      /* Bold Black */
#define BOLDRED     "\033[1m\033[31m"      /* Bold Red */
#define BOLDGREEN   "\033[1m\033[32m"      /* Bold Green */
#define BOLDYELLOW  "\033[1m\033[33m"      /* Bold Yellow */
#define BOLDBLUE    "\033[1m\033[34m"      /* Bold Blue */
#define BOLDMAGENTA "\033[1m\033[35m"      /* Bold Magenta */
#define BOLDCYAN    "\033[1m\033[36m"      /* Bold Cyan */
#define BOLDWHITE   "\033[1m\033[37m"      /* Bold White */

/*****************************************************************************************
 * @description: 
 * @reference: 
 */
class EDFMap
{
    private:
    cv::Mat* _edfmap;
    cv::Mat _edfmap_A;
    cv::Mat _edfmap_B;
    cv::Mat element;        // 用于腐蚀的核


    public:
    double _map_origin_x;   // 地图原点
    double _map_origin_y;   // 地图原点
    double _map_size_x;     // 地图尺寸
    double _map_size_y;     // 地图尺寸
    double _map_resolution; // 地图分辨率
    double _mini_dist;      // 用于处理距离场为0的情况
    
    // cv::Mat* _edfmap;
    // cv::Mat _edfmap_A;
    // cv::Mat _edfmap_B;
    EDFMap() = default;
    ~EDFMap(){};

    // 读取地图二值地图 生成EDF地图 用于优化求解的测试
    bool readMap(std::string _image_address, int erode_kernel_size_) {
        // _image_address = std::string("/home/zwt/catkin_ws/src/grid_path_searcher/map/map1.png");
        cv::Mat orign_img = cv::imread(_image_address,cv::IMREAD_GRAYSCALE);
        if (orign_img.empty()) {
            std::cout << "Error opening image" << std::endl;
            return false;        
        }
        //读取图像二值化
        if (erode_kernel_size_ > 0) {
            element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(erode_kernel_size_, erode_kernel_size_));
            cv::erode(orign_img, orign_img, element);
        }
        cv::Mat map_img;
        cv::threshold(orign_img, map_img, 128, 255,cv::THRESH_BINARY);
        cv::distanceTransform(map_img,_edfmap_A,cv::DIST_L2,cv::DIST_MASK_3);
        _edfmap_A = _edfmap_A * _map_resolution;
        _map_origin_x = 0.0;
        _map_origin_y = 0.0;
        _edfmap = &_edfmap_A;
        return true;
    }


    // 设定地图映射参数 原点 | 尺寸 | 分辨率 | 最小距离(用于处理距离场为0的情况)
    void setMapParam(double map_size_x, double map_size_y, double map_resolution, double mini_dist) {
        _map_origin_x = map_size_x / 2;
        _map_origin_y = map_size_y / 2;
        _map_size_x = map_size_x;
        _map_size_y = map_size_y;
        _map_resolution = map_resolution;
        _mini_dist = mini_dist;
        // in opencv Mat :row == heigh == Point.y;col == width == Point.x;Mat::at(Point(x, y)) == Mat::at(y,x)
        int map_col = std::ceil(_map_size_x / _map_resolution);
        int map_row = std::ceil(_map_size_y / _map_resolution);
        _edfmap_A = cv::Mat(map_col, map_row, CV_32FC1, cv::Scalar(255));
        _edfmap_B = cv::Mat(map_col, map_row, CV_32FC1, cv::Scalar(255));
        _edfmap = &_edfmap_A;
    }
    // 更新EDF地图
    inline void setMap(const cv::Mat &map) {
        if (_edfmap == &_edfmap_A) {
            _edfmap_B = map.clone();
            _edfmap = &_edfmap_B;
        } else {
            _edfmap_A = map.clone();
            _edfmap = &_edfmap_A;
        }
    }
    // 更新EDF地图
    inline void setMap(const cv::Mat *map) {
        if (_edfmap == &_edfmap_A) {
            _edfmap_B = map->clone();
            _edfmap = &_edfmap_B;
        } else {
            _edfmap_A = map->clone();
            _edfmap = &_edfmap_A;
        }
    }

    inline void getDistGrad(double xpos,double ypos,double &dist,double &xgrad,double &ygrad) const {
        int _idx = std::floor((xpos + _map_origin_x) / _map_resolution);
        int _idy = std::floor((ypos + _map_origin_y) / _map_resolution);
        dist = getDist(_idx,_idy);
        double distxN = getDist(_idx - 1,_idy);
        double distxP = getDist(_idx + 1,_idy);
        double distyN = getDist(_idx,_idy - 1);
        double distyP = getDist(_idx,_idy + 1);
        xgrad = ((dist - distxN) + (distxP - dist)) / 2.0 / _map_resolution;
        ygrad = ((dist - distyN) + (distyP - dist)) / 2.0 / _map_resolution;
    }

    void getDistGrad(Eigen::Vector2d pos,double &dist,Eigen::Vector2d &grad) const {
        int _idx = std::floor((pos(0) + _map_origin_x) / _map_resolution);
        int _idy = std::floor((pos(1) + _map_origin_y) / _map_resolution);
        dist = getDist(_idx,_idy);
        double distxN = getDist(_idx - 1,_idy);
        double distxP = getDist(_idx + 1,_idy);
        double distyN = getDist(_idx,_idy - 1);
        double distyP = getDist(_idx,_idy + 1);
        grad(0) = ((dist - distxN) + (distxP - dist)) / 2.0 / _map_resolution;
        grad(1) = ((dist - distyN) + (distyP - dist)) / 2.0 / _map_resolution;
    }

    inline double getDist(int idx, int idy) const {
        if (idx < 0 || idx >= _edfmap->cols || idy < 0 || idy >= _edfmap->rows) {
            if (idx < 0) idx = 0;
            if (idx >= _edfmap->cols) idx = _edfmap->cols - 1;
            if (idy < 0) idy = 0;
            if (idy >= _edfmap->rows) idy = _edfmap->rows - 1;
            //// 预定义地图范围外的距离都是 0 (这样粗暴处理不合理) 所以收缩超出范围的地图变成地图边界的距离场
        }
        double dist = _edfmap->at<float>(idy, idx);
        if (dist < DOUBLE_ZERO)
            return _mini_dist;
        else // MATLAB中如此计算所有距离场均 >= mini_dist
            return dist + _mini_dist;
    }
};
#endif // EDFMAP_HPP_