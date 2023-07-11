/*
 * @Author: wentao zhang && zwt190315@163.com
 * @Date: 2023-07-07
 * @LastEditTime: 2023-07-11
 * @Description: trajectory optimization class test
 * @reference: 
 * 
 * Copyright (c) 2023 by wentao zhang, All Rights Reserved. 
 */

#include <gtest/gtest.h>
#include <iostream>
#include <Eigen/Eigen>
#include <nontrajopt/nontrajopt.hpp>

// using namespace NonTrajOptSpace;

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    // ros::init (argc,argv, "tester" );
    // ros::NodeHandle nh;
    return RUN_ALL_TESTS();
}

TEST(NonTrajOptTest, updateOptValue) {
    NonTrajOpt nontrajopt;
    
}