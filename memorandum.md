# TODO List

## Navigation
### fast_navigation

使用MATLAB优化出来的轨迹，保存在`.csv`中，读入`swift_planner`中进行跟踪
- [x] 轨迹的读入`Readcsv`类
- [ ] 离线轨迹的可视化
- [ ] 如何解决跟踪误差偏大造成失控翻车
- [ ] 坐标系的转化 SLAM <=> Robot

#### Replanning
思路：先检查当前Odometry是否安全/根据当前odometry计算1s的轨迹是否安全，危险直接停机；判断当前pose在期望轨迹上的位置，并检查之后的轨迹是否安全，从不安全的Segment开始，重新search，填补到期望轨迹向量中。要将lazykinoprm中的pathStateSets和fast_navigation分离开来，fast_navigation要有自己的一套waypoint数据。
1. `rcvOdomCallBack` 函数中的重规划判断逻辑
- [ ] waypoint date struct
- [x] safe check and replan

### lazykinoprm
- [x] `TrajectoryCheck` 单纯轨迹的位置安全性检查
- [x] `PathStatesSetCheck` 轨迹/路点的安全检查并记录检查状态


### nontrajopt
- [x] Polynomial Trajectory as `segment.hpp`
- [x] 优化问题降维需要构建一个线性方程组$Ax=b$并且求解，其中的系数矩阵$A$是一个带状矩阵,确定这个矩阵的上下界`lowerBd`和`upperBd`,可以使用带状矩阵的LU分解快速求解方程组

#### OSQP Solver
- [x] updateQ,updateAeqbeq Debug
- [ ] ERROR: non-convex 这可怎么办呢，这咋解呀
```
ERROR in LDL_factor: Error in KKT matrix LDL factorization when computing the nonzero elements. The problem seems to be non-convex
ERROR in osqp_setup: KKT matrix factorization.
The problem seems to be non-convex
```
#### NLopt Solver
- [ ] 减少条件,先用NLopt求解器解相同的QP问题
    - [x] updateOptAxb Debug
    - [x] update Ax=b solve Debug
    - [x] `set_min_objective` Debug
    - [ ] `NLoptCost` Debug ID:`f070e92`
    - [ ] `NLoptSolve()` Debug
- [x] edf map Debug

### BUGs
#### Big!!!
- [x] start swift planner 第一个数据会有一个(0,0,0)的期望，让机器人猛地一顿  
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

### Target Trajectory
- [ ] timestate 序列变成可调,由轨迹跟踪的`nav_seq`控制

### NMPC
- [ ] Oval Constrain: 添加线速度与方向角的椭圆形约束
- [ ] Velocity Penalty: 添加机器人线速度与角速度的安全性代价约束

# TEST List

##
- [ ] CLF-TRO build and roslaunch


## Search

## Opt
- [ ] mini jerk smoothcost ?
## NMPC

# Important Version List

