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
`e6ac6a5`: **feat** GoalPoint Obstacle Infeasible Reset;**fix** get Pose SDF;
在规划过程中，局部地图无法判断Goal的状态，随着地图更新可能造成终点是Obstacle Infeasible，在Search准备阶段进行判断并Reset，在终点附近找一个合适的Goal，搜索一地区域找个距离最近的，但是这样造成问题，距离障碍物太近，搜索轨迹的时候几乎从各个方向都会碰的障碍物，所以使用sdf信息额外考虑一个threshold
2023-7-3  
`f317360`: **feat** swift local map replanning;  
可以使用局部地图，进行lazykinoprm的路径规划了，基本能够完成规划任务(为了保证稳定性,机器人走得比较慢，标准规划速度的`0.2`)
2023-7-29
`ff7a237`: **debug** swift realtime navigation
使用局部地图，在初始搜索的后，使用OSQP(全局优化初始解)+NLopt(局部轨迹优化)，进行完整的导航工作流程(标准规划速度`0.4`，OSQP成功率`50%`，NLopt成功率`<10%`，如同鸡肋，不能复现MATLAB中的效果)
2023/7/31
`45b49e2`: **pref** swift tracking offline trajectory
2023/12/10
`cc383be`: **fix** swift navigation fix solve bug

2023/12/21
`fb38a63`: **pref** all project for release
可以实现稳定的轨迹非线性优化，整个`swift_navigation`实现预期的完整功能
1. 在线规划接收局部地图，并更新2D障碍图和SDF地图
2. 在2D障碍物地图的基础上使用`lazykinoprm`搜索轨迹
3. 在搜索轨迹的基础上使用`nontrajopt`优化轨迹
4. 根据当前`odometry`计算Horizon内的轨迹传递给MPC跟踪