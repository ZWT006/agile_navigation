# Important Version List

`43ee902`: Virtual Global Map; LazyKinoPRM Search; NMPC Trajectory Tracking;  
从二值图片获得虚拟地图并且全局更新，使用LazyKinoPRM搜索得到时空轨迹，并根据odometry进行时空轨迹的跟踪。**注意**：每设置一次*Goal Point*就触发一次全局搜索。  
启动文件：完全启动后，使用鼠标选择终点位置即可实现导航  
```
# terminal·1 启动虚拟地图生成&可视化节点
roslaunch grid_path_searcher mapworld.launch
# terminal·2 启动fast-navigation导航管理节点
roslaunch fast_navigation fast_planner.launch
# terminal·n 启动legged control tracking branch
```

2023-6-15
``: **feat** GoalPoint Obstacle Infeasible Reset;**fix** get Pose SDF;
在规划过程中，局部地图无法判断Goal的状态，随着地图更新可能造成终点是Obstacle Infeasible，在Search准备阶段进行判断并Reset，在终点附近找一个合适的Goal，搜索一地区域找个距离最近的，但是这样造成问题，距离障碍物太近，搜索轨迹的时候几乎从各个方向都会碰的障碍物，所以使用sdf信息额外考虑一个threshold