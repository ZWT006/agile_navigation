# swift_nav

## 介绍
华中科技大学-人工智能与自动化学院-Coralab实验室，四足机器人未知环境下快速运动规划与控制ROS功能包

## 软件架构
**核心功能**:整个实时导航的功能实现，包含前置感知：获取SLAM信息，地图处理；导航规划：路径初搜索，分段轨迹优化，轨迹跟踪
`fast_navigation`:包含整个导航框架,进行整个导航工程的全部管理
`lazykinoprm`:考虑动力学的轨迹搜索
`nontrajopt`:非线性轨迹优化
**辅助功能**:对地图的处理，rviz的可视化功能，获取导航目标点等辅助功能，以下功能包使用开源代码，并对某些功能包的代码进行部分修改，功能包来源
`grid_path_search`:https://github.com/chunyang-zhang/grid_path_searcher/
`rviz_plugins`:
`waypoint_generator`:https://github.com/epan-utbm/waypoint_generator


## 安装教程
除了本项目的源码之外，还有一些与具体实现快速实时导航的子功能模块，并且本功能包的使用依赖于ROS，**安装教程**中涉及的代码需要保存在工作空间的`src`目录下，并且在**Ubuntu20.04/ROS noetic**下进行使用，如果使用其它版本**Ubuntu**有该版本对应的**ROS**请自行修改可能存在的问题。本项目的Gitee地址：https://gitee.com/hi-zwt/swift_nav

### 依赖
* OpenCV
* Eigen3
* OSQP
* OSQP-Eigen
* NLopt

1. 直接安装`Eigen`：`sudo apt-get install libeigen3-dev`
2. 源码安装`OSQP`：[OSQP/Get Start](https://osqp.org/docs/get_started/)，本工程使用版本为`0.6.3`
    ```
    git clone --recursive -b release-0.6.3 https://github.com/oxfordcontrol/osqp.git
    cd osqp
    mkdir build && cd build
    cmake .. 
    make
    sudo make install
    ```
3. 源码安装`osqp-eigen`:[osqp-eigen](https://robotology.github.io/osqp-eigen/)
    ```
    git clone https://github.com/robotology/osqp-eigen.git
    cd osqp-eigen
    mkdir build && cd build
    cmake .. 
    make
    sudo make install
    ```
4. 源码安装`nlopt`：[NLopt Documentation](https://nlopt.readthedocs.io/en/latest/)，本工程使用版本为`2.7.1`，**注意**使用指令`sudo apt-get install libnlopt.dev`可能造成使用错误，无法正常求解
    ```
    git clone --recursive -b v2.7.1 https://github.com/stevengj/nlopt.git
    cd nlopt
    mkdir build
    cd build
    cmake ..
    make
    sudo make install
    ```
### Planner
1. 下载源码并切换到`release`分支，在工作空间下使用`catkin build fast_navigation`编译代码
    ```
    git clone --recursive -b release https://gitee.com/hi-zwt/swift_nav.git
    ```
### Control
1. legged control: https://gitee.com/hi-zwt/legged_control 切换到`tracking`分支
    ```
    git clone --recursive -b tracking https://gitee.com/hi-zwt/legged_control
    ```
2. 安装legged control的相关依赖
3. udpros_bridge: https://gitee.com/coralab/udpros-bridge SLAM与NMPC设备间通讯，实物部署需要(选择安装，感知与控制在同一设备不需要此软件包)
    ```
    git clone https://gitee.com/coralab/udpros-bridge.git
    ```

### SLAM 
无需实物部署不需要下载，或根据需求仿真雷达(1.)，里程计(6.)，建图(7.)其余为Mid360 lidar相关驱动，关于雷达驱动和建图软件包的使用参考具体的文档，整个规划工程的详细使用参考[使用说明](./Usage.md)
1. livox lidar 仿真文件[livox_laser_simulation](https://github.com/Livox-SDK/livox_laser_simulation)
2. livox lidar SDK [Livox-SDK](https://github.com/Livox-SDK/Livox-SDK)
3. livox lidar SDK [Livox-SDK2](https://github.com/Livox-SDK/Livox-SDK2)
4. livox ros 驱动[livox_ros_driver](https://github.com/Livox-SDK/livox_ros_driver)
5. livox ros 驱动[livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2)
6. Lidar SLAM [FAST_LIO](https://github.com/hku-mars/FAST_LIO)
7. ESDF map [FIESTA](https://github.com/ZWT006/FIESTA)修改版


### Simulation 
仿真版本可以只安装 `swift_nav` 和 `legged_control`
`legged_control` : `tracking` 分支  
`swift_nav` : `release`分支  
安装编译无误后，分别开启以下终端 首先`source ~/motion_ws/devel/setup.bash`(注意工作空间路径)  

1. 启动Gazebo仿真环境
```
export ROBOT_TYPE=a1
roslaunch legged_unitree_description empty_world.launch
```
2. 加载legged_controller控制器，作弊模式(使用Gazebo仿真器获得状态)
```
roslaunch legged_controllers load_controller.launch cheater:=true
```
3. 开启遥控器控制节点(可选)
```
roslaunch legged_controllers joy_teleop.launch
```
4. 开启legged_controller控制器(输入任何commond机器人就会运动)
```
rosservice call /controller_manager/switch_controller "start_controllers: ['controllers/legged_controller']
stop_controllers: ['']
strictness: 0
start_asap: false
timeout: 0.0" 
```
5. 输入机器人步态，在(2.)端口
```
flying_trot
```
6. 加载虚拟地图
```
roslaunch grid_path_searcher mapworld.launch
```
7. 加载规划器
```
roslaunch fast_navigation swift_planner.launch
```
8. 设定目标点: 使用 3D goal 在(6.)的rviz界面设定目标点 or 手动发布 [x,y,q(deg)]
```
rosrun fast_navigation pub_goalpose 0.0 0.0 0.0
```

### realworld
真实世界部署还需要安装SLAM相关的驱动和功能包，并在仿真工作流程的基础上修改(2.)环节
2. 加载legged_controller控制器使用非作弊模式(使用状态估计器获得状态)
```
roslaunch legged_controllers load_controller.launch cheater:=false
```

## 重要参考

1.  HKUST-Aerial-Robotics [Fast_Planner](https://github.com/HKUST-Aerial-Robotics/Fast-Planner)
B. Zhou, F. Gao, L. Wang, C. Liu and S. Shen, "Robust and Efficient Quadrotor Trajectory Generation for Fast Autonomous Flight," in IEEE Robotics and Automation Letters, vol. 4, no. 4, pp. 3529-3536, Oct. 2019, doi: 10.1109/LRA.2019.2927938.
2.  ZJU-FAST-Lab [GCOPTER](https://github.com/ZJU-FAST-Lab/GCOPTER)
Z. Wang, X. Zhou, C. Xu and F. Gao, "Geometrically Constrained Trajectory Optimization for Multicopters," in IEEE Transactions on Robotics, vol. 38, no. 5, pp. 3259-3278, Oct. 2022, doi: 10.1109/TRO.2022.3160022.