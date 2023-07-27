# TODO List

## Navigation
### fast_navigation

使用MATLAB优化出来的轨迹，保存在`.csv`中，读入`swift_planner`中进行跟踪
- [x] 轨迹的读入`Readcsv`类
- [ ] 离线轨迹的可视化
- [ ] 如何解决跟踪误差偏大造成失控翻车
- [ ] 坐标系的转化 SLAM <=> Robot
- [ ] 手动输入 Goal (非GUI输入)
- [ ] 精简代码结构和功能 以完成基础功能为主

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
- [x] Traj 离散化参数计算 Debug 离散段数量/dt/离散状态计算
- [x] Time 多次幂计算/exp()/log()
- [ ] 角度q变化的光滑处理
- [ ] 参考时间计算以及初末时间缩放
- [ ] QP优化和NLopt优化如何区分 长距离 QP + 短距离 NLopt?

#### OSQP Solver
- [x] updateQ,updateAeqbeq Debug
- [x] ERROR: non-convex 这可怎么办呢，这咋解呀  
搞定了，是$Q$矩阵算错了，自己写的C++代码的问题
```
ERROR in LDL_factor: Error in KKT matrix LDL factorization when computing the nonzero elements. The problem seems to be non-convex
ERROR in osqp_setup: KKT matrix factorization.
The problem seems to be non-convex
```
- [x] ~~add waypoints states inequality constraints~~ 还是不要的好 效果差
- [ ] maybe 可以把求解的输出给关了

#### NLopt Solver
- [x] updateOptAxb Debug
- [x] update Ax=b solve Debug
- [x] `set_min_objective` Debug : 编译通过运行出错/源码安装NLopt解决
- [x] `NLoptCost` Debug ID:`f070e92` 
    - [x] smoCost 基本一致 Gradc / Gradt
    - [x] timeCost 基本一致
    - [x] obsCost 反正Cost都Debug了 我不管!
- [x] `nonCost` Debug ID: `f34f159` for all Debug data save as `.csv` compared with MATLAB
- [x] `OSQPSolve()` Debug
- [ ] `NLoptSolve()` Debug
- [x] edf map Debug
- [ ] 奇异矩阵的线性最小二乘解
- [ ] MATLAB 对比一下是不是一直都是满秩的
### BUGs
#### Big!!!
- [x] start swift planner 第一个数据会有一个(0,0,0)的期望，让机器人猛地一顿  
修改`_REACH_GOAL`初始就跟踪一个平均期望点
- [ ] `tracking.NavSeqFixed(tracking._local_odom);` 造成机器人原地偏移, Odometry根据机器人的踏步在漂
- [ ] replanning 的逻辑有些问题，貌似一直从 currPose 开始重规划不行，或者说得在别的地方一直更新 currPose
- [x] 现在的逻辑是从距离障碍物最近的轨迹点开始重规划,可能造成多走冤枉路的情况出现,这个可以考虑比较一下 障碍物的段索引和当前跟踪的段索引，找一个阈值来确定重规划的段索引
- [ ] `swift_planner`第一次搜索失败后再重设终点会直接死掉
#### Warry

#### Easy
- [ ] Lidar SLAM 的 Frame 和 A1 Body的偏移问题  
直接在`rcvOdomCallback()`中添加一个坐标系转化的关系

## SLAM

### FIESTA
- [x] Local Height PCL Update => `local_update` bool flag
- [x] ESDF Update Switch => `esdf_update` bool flag

## legged control
- [ ] merge low state publish & nav seq subscribe

### Target Trajectory
- [ ] timestate 序列变成可调,由轨迹跟踪的`nav_seq`控制

### NMPC
- [ ] Oval Constrain: 添加线速度与方向角的椭圆形约束
- [ ] Velocity Penalty: 添加机器人线速度与角速度的安全性代价约束

## UDP
- [ ] udp send ros msg
- [ ] udp receive datas
- [ ] publish msg node

# TEST List

## Navigation
- [ ] 10Hz or 5Hz and gait frequancy make different
- [ ] Time State givened by navigation
## bspline
- [ ] CLF-TRO build and roslaunch
- [ ] CLF-TRO State Trajectory


## Search
~~everything is OK~~对不起不该这样说
- [x] add obs_map and fat_map 避免机器人本体形状的影响
- [x] fix bug about obsmap update, add resetLocalObstmap to avoid obstacle pcl accumulative
- [ ] search trajectory smooth 对比 MATLAB 与 C++ 中的 cost 数量级

## Opt
- [ ] mini jerk smoothcost ?
- [ ] dynCost,obsCost,timCost,ovaCost weights

## NMPC

# Important Version List

