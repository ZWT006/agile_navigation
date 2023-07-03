# TODO List

## Navigation
### fast_navigation

#### Replanning
思路：先检查当前Odometry是否安全/根据当前odometry计算1s的轨迹是否安全，危险直接停机；判断当前pose在期望轨迹上的位置，并检查之后的轨迹是否安全，从不安全的Segment开始，重新search，填补到期望轨迹向量中。要将lazykinoprm中的pathStateSets和fast_navigation分离开来，fast_navigation要有自己的一套waypoint数据。
1. `rcvOdomCallBack` 函数中的重规划判断逻辑
- [ ] waypoint date struct
- [ ] safe check and replan

BUGs
- [] start swaft planner 第一个数据会有一个(0,0,0)的期望，让机器人猛地一顿
- [] replanning 的逻辑有些问题，貌似一直从 currPose 开始重规划不行，或者说得在别的地方一直更新 currPose
- [] 现在的逻辑是从距离障碍物最近的轨迹点开始重规划,可能造成多走冤枉路的情况出现,这个可以考虑比较一下 障碍物的段索引和当前跟踪的段索引，找一个阈值来确定重规划的段索引
### lazykinoprm
- [x] `TrajectoryCheck` 单纯轨迹的位置安全性检查
- [ ] `PathStatesSetCheck` 轨迹/路点的安全检查并记录检查状态


### nontrajopt
1. Polynomial Trajectory 
2. OSQP Solver
#### NLopt Solver

## SLAM

### FIESTA
- [ ] Local Height PCL Update => `local_update` bool flag
- [ ] ESDF Update Switch => `esdf_update` bool flag

## legged control


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