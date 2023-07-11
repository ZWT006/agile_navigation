# swift_nav

## 介绍
HUST AIA Coralab Aggressive Navigation ROS/C++ Code
It's a fast realtime navigation ros package for quadraped robot swift motion in unknow environment.

## 软件架构

## 代码说明
**核心功能**整个实时导航的功能实现，包含前置感知：获取SLAM信息，地图处理；导航规划：路径初搜索，分段轨迹优化，轨迹跟踪
`fast_navigation`:包含整个导航框架,进行整个导航工程的全部管理
`lazykinoprm`:考虑动力学的轨迹搜索
`nontrajopt`:非线性轨迹优化
**辅助功能**对地图的处理，rviz的可视化功能，获取导航目标点等辅助功能，以下功能包使用开源代码，并对某些功能包的代码进行部分修改
`grid_path_search`:
`rviz_plugins`:
`waypoint_generator`:


## 安装教程
除了本项目的源码之外，还有一些与具体实现快速实时导航的子功能模块，并且本功能包的使用依赖于ROS，**安装教程**中涉及的代码需要保存在工作空间的`src`目录下，并且在**Ubuntu20.04/ROS noetic**下进行使用，如果使用其它版本**Ubuntu**有该版本对应的**ROS**请自行修改可能存在的问题。

### 源码

1.  本项目的Gitee地址：[Hi-ZWT/swift_nav](https://gitee.com/hi-zwt/swift_nav)
```
git clone https://gitee.com/hi-zwt/swift_nav
```
2.  源码依赖`nlopt`：`sudo apt-get install ros_noetic_nlopt`
3.  源码依赖`Eigen`：`sudo apt-get install libeigen3-dev`
4.  源码依赖`OSQP`：[OSQP/Get Start](https://osqp.org/docs/get_started/)

### Control
1. legged control [reference](https://gitee.com/hi-zwt/legged_control) 切换到`tracking`分支
```
git clone https://gitee.com/hi-zwt/legged_control
```
2. 安装legged control的相关依赖

### SLAM

1. livox lidar 仿真文件[livox_laser_simulation](https://github.com/Livox-SDK/livox_laser_simulation)(可选)
2. livox lidar [SDK1](https://github.com/Livox-SDK/Livox-SDK)
3. livox lidar [SDK2](https://github.com/Livox-SDK/Livox-SDK2)
4. livox ros [driver1](https://github.com/Livox-SDK/livox_ros_driver)
5. livox ros [driver2](https://github.com/Livox-SDK/livox_ros_driver2)
6. LIdar SLAM [FAST_LIO](https://github.com/hku-mars/FAST_LIO)

## [使用说明](./Usage.md)

### Simulation 
仿真版本可以只安装 `swift_nav` 和 `legged_control`
`legged_control` : tracking 分支  
`swift_nav` : `f317360`分支  
安装编译无误后，分别开启以下终端 首先`source ~/motion_ws/devel/setup.bash`(注意工作空间路径)  
#### legged control 工作流程
1. export ROBOT_TYPE=a1
roslaunch legged_unitree_description empty_world.launch

2. export ROBOT_TYPE=a1
roslaunch legged_controllers load_controller.launch cheater:=true
ps:输入步态: flying_trot

3. 开启运动
rosservice call /controller_manager/switch_controller "start_controllers: ['controllers/legged_controller']
stop_controllers: ['']
strictness: 0
start_asap: false
timeout: 0.0" 

4. 开启虚拟地图
roslaunch grid_path_searcher mapworld.launch

5. 开启规划
roslaunch fast_navigation swift_planner.launch

PS: 使用 3D goal 设定目标点

## [备注](./memorandum.md)

## [开发记录](./Changelog.md)
## 参与贡献

1.  Fork 本仓库
2.  新建 Feat_xxx 分支
3.  提交代码
4.  新建 Pull Request


## 特技

1.  使用 Readme\_XXX.md 来支持不同的语言，例如 Readme\_en.md, Readme\_zh.md
2.  Gitee 官方博客 [blog.gitee.com](https://blog.gitee.com)
3.  你可以 [https://gitee.com/explore](https://gitee.com/explore) 这个地址来了解 Gitee 上的优秀开源项目
4.  [GVP](https://gitee.com/gvp) 全称是 Gitee 最有价值开源项目，是综合评定出的优秀开源项目
5.  Gitee 官方提供的使用手册 [https://gitee.com/help](https://gitee.com/help)
6.  Gitee 封面人物是一档用来展示 Gitee 会员风采的栏目 [https://gitee.com/gitee-stars/](https://gitee.com/gitee-stars/)
