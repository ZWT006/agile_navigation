# TODO List

## Navigation
### fast_navigation

#### Replanning
思路：先检查当前Odometry是否安全/根据当前odometry计算1s的轨迹是否安全，危险直接停机；判断当前pose在期望轨迹上的位置，并检查之后的轨迹是否安全，从不安全的Segment开始，重新search，填补到期望轨迹向量中。要将lazykinoprm中的pathStateSets和fast_navigation分离开来，fast_navigation要有自己的一套waypoint数据。
1. `rcvOdomCallBack` 函数中的重规划判断逻辑
- [ ] waypoint date struct
- [x] safe check and replan

### lazykinoprm
- [x] `TrajectoryCheck` 单纯轨迹的位置安全性检查
- [x] `PathStatesSetCheck` 轨迹/路点的安全检查并记录检查状态


### nontrajopt
1. Polynomial Trajectory 
2. OSQP Solver
#### NLopt Solver

### BUGs
#### Big!!!
- [x] start swaft planner 第一个数据会有一个(0,0,0)的期望，让机器人猛地一顿  
修改`_REACH_GOAL`初始就跟踪一个平均期望点
- [ ] `tracking.NavSeqFixed(tracking._local_odom);` 造成机器人原地偏移, Odometry根据机器人的踏步在漂
- [ ] replanning 的逻辑有些问题，貌似一直从 currPose 开始重规划不行，或者说得在别的地方一直更新 currPose
- [x] 现在的逻辑是从距离障碍物最近的轨迹点开始重规划,可能造成多走冤枉路的情况出现,这个可以考虑比较一下 障碍物的段索引和当前跟踪的段索引，找一个阈值来确定重规划的段索引

#### Warry

#### Easy
- [ ] Lidar SLAM 的 Frame 和 A1 Body的偏移问题  
直接在`rcvOdomCallback()`中添加一个坐标系转化的关系

## SLAM

### FIESTA
- [x] Local Height PCL Update => `local_update` bool flag
- [x] ESDF Update Switch => `esdf_update` bool flag

## legged control


# Important Version List

