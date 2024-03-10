# swift_nav

## Introduction
Huazhong University of Science and Technology - School of Artificial Intelligence and Automation - Coralab Lab, ROS package for *Agile and Safe Trajectory Planning for Quadruped Navigation with Motion Anisotropy Awareness*

## Framework
**Key Function**:The entire real-time navigation implementation, including: coarse path search, segmented trajectory optimization, trajectory tracking;
`fast_navigation`:包含整个导航框架,进行整个导航工程的全部管理
`lazykinoprm`:Kinodynamic Trajectory Generation
`nontrajopt`:Nonlinear Trajectory Optimization
**Auxiliary Functions**:visualization of rviz, acquisition of navigation target points and other auxiliary functions, the following packages use open source code and partially modification.
`grid_path_search`:https://github.com/chunyang-zhang/grid_path_searcher/
`rviz_plugins`:
`waypoint_generator`:https://github.com/epan-utbm/waypoint_generator


## Install
In addition to the source code of this project, there are a number of sub-functional modules related to the specific implementation of the real-time navigation framework, and all of these are used under the **Ubuntu20.04/ROS noetic**.

### Dependencies
* OpenCV
* Eigen3
* OSQP
* OSQP-Eigen
* NLopt

1. Install in Ubuntu Terminal `Eigen`：`sudo apt-get install libeigen3-dev`
2. Install by source code `OSQP`：[OSQP/Get Start](https://osqp.org/docs/get_started/)，version `0.6.3`
    ```
    git clone --recursive -b release-0.6.3 https://github.com/oxfordcontrol/osqp.git
    cd osqp
    mkdir build && cd build
    cmake .. 
    make
    sudo make install
    ```
3. Install by source code `osqp-eigen`:[osqp-eigen](https://robotology.github.io/osqp-eigen/)
    ```
    git clone https://github.com/robotology/osqp-eigen.git
    cd osqp-eigen
    mkdir build && cd build
    cmake .. 
    make
    sudo make install
    ```
4. Install by source code `nlopt`：[NLopt Documentation](https://nlopt.readthedocs.io/en/latest/)，versioin `2.7.1`，NOTE** Using the command `sudo apt-get install libnlopt.dev` may result in error.
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
1. Clone source code and switch to `release` branch and build in workspace by `catkin build fast_navigation`
    ```
    git clone --recursive -b release https://github.com:ZWT006/agile_navigation.git
    ```
### Control
The controller here uses [legged_control](https://github.com/qiayuanl/legged_control), thanks to this researcher's open source, please go to https://github.com/qiayuanl/legged_control for details. We've made some modifications for tracking
1. Clone legged control and switch to `tracking` branch
    ```
    git clone --recursive -b tracking https://github.com:ZWT006/legged_control.git
    ```
2. Install dependances

### Simulation 

1. Start Gazebo Simulation
    ```
    export ROBOT_TYPE=a1
    roslaunch legged_unitree_description empty_world.launch
    ```
2. Load legged_controller，either cheater:=true or false will work 
    ```
    roslaunch legged_controllers load_controller.launch cheater:=true
    ```
3. load remote node (optional)
    ```
    roslaunch legged_controllers joy_teleop.launch
    ```
4. Start legged_controller
    ```
    rosservice call /controller_manager/switch_controller "start_controllers: ['controllers/legged_controller']
    stop_controllers: ['']
    strictness: 0
    start_asap: false
    timeout: 0.0" 
    ```
5. Input gait in (2.) terminal. Noting that low-frequency gait may lead to unstable high-speed motion
    ```
    flying_trot
    ```
6. launch virtual map
    ```
    roslaunch grid_path_searcher mapworld.launch
    ```
7. launch planner
    ```
    roslaunch fast_navigation swift_planner.launch
    ```
8. Set goal point: using 3D goal in (6.)' rviz interface or manually post [x,y,q(deg)].
    ```
    rosrun fast_navigation pub_goalpose 0.0 0.0 0.0
    ```

## Realworld
Real-world deployments also require the installation of SLAM-related drivers and packages, depending on the situation. The devices we use are described in our paper.

## Key Reference

1.  HKUST-Aerial-Robotics [Fast_Planner](https://github.com/HKUST-Aerial-Robotics/Fast-Planner)
B. Zhou, F. Gao, L. Wang, C. Liu and S. Shen, "Robust and Efficient Quadrotor Trajectory Generation for Fast Autonomous Flight," in IEEE Robotics and Automation Letters, vol. 4, no. 4, pp. 3529-3536, Oct. 2019, doi: 10.1109/LRA.2019.2927938.
2.  ZJU-FAST-Lab [GCOPTER](https://github.com/ZJU-FAST-Lab/GCOPTER)
Z. Wang, X. Zhou, C. Xu and F. Gao, "Geometrically Constrained Trajectory Optimization for Multicopters," in IEEE Transactions on Robotics, vol. 38, no. 5, pp. 3259-3278, Oct. 2022, doi: 10.1109/TRO.2022.3160022.

**PS**: If you have any questions or suggestions, please feel free to contact `zwt190315@163.com` or `wentaozhang@hust.edu.cn`. Thank you very much.