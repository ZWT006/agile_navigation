# Project Usage

## SLAM
设备：Livox Mid360，A1-NX   
程序：fast_lio(odometry/global pcl),fiesta(probability octomap/3D esdf)

### Instalitation
安装Livox lidar driver 和 SLAM 程序包
#### Livox SDK
该部分安装在`A1-NX`(Jetson-NX为`ARM64`架构)的`Ubuntu18.04`系统，使用`ROS melodic`，使用`catkin_make`编译，在主目录下进行操作
1. SDK安装，建立SDK文件夹
    ```
    mkdir -p ~/sdk_tools
    cd ~/sdk_tools
    ```
2. 编译安装[SDK1](https://github.com/Livox-SDK/Livox-SDK)和[SDK2](https://github.com/Livox-SDK/Livox-SDK2)，请先阅读对应链接的安装指导
    ```
    # clone SDK1
    git clone https://github.com/Livox-SDK/Livox-SDK.git
    cd Livox-SDK

    # if AMD machine
    cd build && cmake ..
    make
    sudo make install

    # else if ARM 32 machine
    cd build && \
    cmake .. -DCMAKE_SYSTEM_NAME=Linux -DCMAKE_C_COMPILER=arm-linux-gnueabi-gcc -DCMAKE_CXX_COMPILER=arm-linux-gnueabi-g++
    make
    sudo make install

    # else if ARM 64 machine
    cd build && \
    cmake .. -DCMAKE_SYSTEM_NAME=Linux -DCMAKE_C_COMPILER=aarch64-linux-gnu-gcc -DCMAKE_CXX_COMPILER=aarch64-linux-gnu-g++
    make
    sudo make install

    # clone SDK2
    cd .. # return sdk_tools directory
    git clone https://github.com/Livox-SDK/Livox-SDK2.git
    # build 
    cd ./Livox-SDK2/
    mkdir build
    cd build
    cmake .. && make -j
    sudo make install
    ```


#### SLAM ROS packages
1. 建立SLAM工作空间
    ```
    mkdir -p ~/slam_ws/src
    cd ~/slam_ws/src
    catkin_init_workspace
    cd ..
    catkin_make
    ```
2. 安装编译[livox_ros_driver](https://github.com/Livox-SDK/livox_ros_driver)
    ```
    cd ~/slam_ws/src
    git clone https://github.com/Livox-SDK/livox_ros_driver.git
    cd ..
    catkin_make
    ```
3. 安装编译[livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2)
    ```
    cd ~/slam_ws/src
    git clone https://github.com/Livox-SDK/livox_ros_driver2.git
    cd livox_ros_driver2/
    source /opt/ros/noetic/setup.sh
    ./build.sh ROS1
    ```
4. 安装编译[FAST_LIO](https://github.com/hku-mars/FAST_LIO)，应用修改版
    ```
    cd ~/slam_ws/src
    git clone https://github.com/hku-mars/FAST_LIO.git
    cd FAST_LIO
    git submodule update --init
    cd ../..
    catkin_make
    ```
5. 安装编译[FIESTA](git@github.com:ZWT006/FIESTA.git)，功能修改版
    ```
    cd ~/slam_ws/src
    git clone https://github.com/ZWT006/FIESTA.git
    cd ..
    catkin_make
    ```

### Running
在个人电脑上进行仿真测试的应用以及在A1实机平台上进行测试

#### SDK/Driveer 测试
1. 根据livox Mid360 雷达IP和本机IP配置文件  
A1-NX网口IP为`192.168.11.100`  
Mid360网口IP为`192.168.11.156`(可以根据用户手册使用LivoxViewer进行修改)
配置`slam_ws/src/livox_ros_driver2/config/MID360_config.json`中IP地址
    ```
    {
    "MID360": {
        "lidar_net_info" : {
        "cmd_data_port": 56100,
        "push_msg_port": 56200,
        "point_data_port": 56300,
        "imu_data_port": 56400,
        "log_data_port": 56500
        }, # 主机 IP
        "host_net_info" : {
        "cmd_data_ip" : "192.168.11.100",
        "cmd_data_port": 56101,
        "push_msg_ip": "192.168.11.100",
        "push_msg_port": 56201,
        "point_data_ip": "192.168.11.100",
        "point_data_port": 56301,
        "imu_data_ip" : "192.168.11.100",
        "imu_data_port": 56401,
        "log_data_ip" : "",
        "log_data_port": 56501
        }
    },
    "lidar_configs" : [
        {# Mid360 的IP 
        "ip" : "192.168.11.156",
        "pcl_data_type" : 1,
        "pattern_mode" : 0,
        }
    ]
    }
    ```
2. 使用rviz查看lidar msg
    ```
    source ~/slam_ws/devel/setup.bash 
    roslaunch livox_ros_driver2 rviz_MID360.launch
    ```

#### FASL_LIO/FIESTA 测试
1. 启动ROS master 并设置使用仿真时间播放数据包
    ```
    # terminal·1
    1. roscore
    # terminal·2
    rosparam set /use_sim_time true
    ## rosbag play -l --clock XXX.bag
    ## eg: rosbag play roof_0415_evening_2023-04-15-10-38-45.bag -l --clock
    ``` 
2. 启动FAST_LIO和FIESTA使用MID360的SLAM启动文件
    ```
    # terminal·3
    source ~/slam_ws/devel/setup.bash
    roslaunch fast_lio mapping_mid360.launch
    # terminal·4
    source ~/slam_ws/devel/setup.bash
    roslaunch fiesta lidar_mid360.launch
    ```
3. 

#### 

## NMPC

```
source ~/motion_ws/devel/setup.bash
```

rosservice call /controller_manager/switch_controller "start_controllers: ['controllers/legged_controller']
stop_controllers: ['']
strictness: 0
start_asap: false
timeout: 0.0" 

flying_trot // 会有 Z height的波动
  switchingTimes
  {
    [0]     0.00
    [1]     0.10
    [2]     0.12
    [3]     0.22
    [4]     0.24

###
修改NMPC优化问题权重 提高速度
task.info
; standard state weight matrix
Q
RH_HFE 权重
RH_KFE 权重

速度期望是零，所以可以调小来提高速度
foot velocity relative to base: 
x 方向