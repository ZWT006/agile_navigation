/*
 * @Author: wentao zhang && zwt190315@163.com
 * @Date: 2023-04-03
 * @LastEditTime: 2023-05-04
 * @Description: 
 * @reference: 
 * 
 * Copyright (c) 2023 by wentao zhang, All Rights Reserved. 
 */
/*
TODO: 修正pose_map 的维度, 以及对应的索引计算； xy只有两个维度,第三个维度给z用还是给yaw用？
*/
#include <lazykinoprm//LazyKinoPRM.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <sstream>
// #define NDEBUG
#include <cassert>
#include <angles/angles.h>

#define ENABLE_DEBUFG_FLAG


using namespace std;
using namespace Eigen;
using namespace cv;


LazyKinoPRM::~LazyKinoPRM()
{
  for (uint32_t map_rows = 0; map_rows < MAX_POSE_MAP_X; map_rows++)
  {
    for (uint32_t map_cols = 0; map_cols < MAX_POSE_MAP_Y; map_cols++)
    {
      for (uint32_t map_depth = 0; map_depth < MAX_POSE_MAP_D; map_depth++)
      {
        delete[] pose_map[map_rows][map_cols][map_depth];
      }
      delete[] pose_map[map_rows][map_cols];
    }
    delete[] pose_map[map_rows];
  }
  delete[] pose_map;
  delete[] obs_map;
  delete[] raw_obs_map;
}

/** 
 * @description:  LazyKinoPRM search 
 * @reference: 
 * @return {*}
 */
bool LazyKinoPRM::search(Eigen::Vector3d start_pos, Eigen::Vector3d start_vel,
                        Eigen::Vector3d start_acc, Eigen::Vector3d goal_pos,
                        Eigen::Vector3d goal_vel,  Eigen::Vector3d goal_acc)
{
  //##################################################################################################################
  // step1:put start and goal into the pose_map
  cout << GREEN <<  "<---------prepare search--------->"   << RESET << endl;
  GridNodePtr mid_node=NULL;
  PointStart = start_pos;
  PointGoal  = goal_pos;
  VelStart   = start_vel;
  VelGoal    = goal_vel;
  AccStart   = start_acc;
  AccGoal    = goal_acc;
  start_pose_index_ = Pose2Index(start_pos);
  goal_pose_index_  = Pose2Index(goal_pos);

  mid_node = Pose2PoseNode(goal_pos);
  if(mid_node==NULL)
  {
    cout << RED << "goal node is not in the map ? " << RESET << endl;
    return true;
  }
  mid_node->setPose(goal_pos,goal_pose_index_);
  mid_node->setType(Goal);

  mid_node = Pose2PoseNode(start_pos);
  if(mid_node==NULL)
  {
    cout << RED << "start node is not in the map ? " << RESET << endl;
    return true;
  }
  mid_node->setPose(start_pos,start_pose_index_);
  mid_node->setType(Start);
  
  cout << "[\033[34mnode\033[0m]start_pose_index_:" << start_pose_index_.transpose() << endl;
  cout << "[\033[34mnode\033[0m]start_pos :" << start_pos.transpose() << endl;
  cout << "[\033[34mnode\033[0m]goal_pose_index_ :" << goal_pose_index_.transpose() << endl;
  cout << "[\033[34mnode\033[0m]goal_pos:" << goal_pos.transpose() << endl;
  cout << "[\033[34mnode\033[0m]vel_factor:" << vel_factor_ << endl;
  cout << "[\033[34mnode\033[0m]ome_factor:" << ome_factor_ << endl;
  cout << "[\033[34mnode\033[0m]DIR_GRID:" << DIR_GRID << endl;
  cout << "[\033[34mnode\033[0m]c_angle:" << c_angle_ << endl;
  // TODO: check if start and goal are in the same voxel

  //##################################################################################################################
  // step2:initialize the LazyKinoPRM search
  SampleOBVP obvpsg(vel_factor_/2,ome_factor_/2,weightR_,time_interval_);
  SampleOBVP obvpmid(vel_factor_,ome_factor_,weightR_,time_interval_);
  astarcloselist.reset();
  astaropenlist.reset();
  // expend the start node
  NodeState newNodeState;
  newNodeState.CurrGridNodePtr = mid_node;
  newNodeState.PareGridNodePtr = nullptr;
  newNodeState.Position = PointStart;
  newNodeState.Velocity = VelStart;
  newNodeState.Acceleration = AccStart;
  goal_cost = getHeuristic(PointStart,PointGoal,&newNodeState);
  // start to start so the obvp is not needed
  astaropenlist.insert(&newNodeState,&newNodeState);
  // mid_node = astarcloselist.insert(mid_node);
  no_path_ = true;
  use_node_num_ = 1;
  iter_num_ = 0;
  abandon_node_num_ = 0;
  search_node_num_ = 0;
  collision_flag = false;
  stop_search_ = false;
  goal_node_listindex_ = 0;
  cout << GREEN <<  "<---------start search--------->"   << RESET << endl;
  //##################################################################################################################
  // step3:LazyKinoPRM search
  // stop: 1) openlist is all extend;2) stop search flag = true;
  while (true)
  {
    iter_num_ ++;
    //3.1:it's time to stop search
    if (stop_search_ || astaropenlist.IsAllExtend() || iter_num_ > iter_num_max_)
    {
      cout << GREEN << "stop search" << RESET << endl;
      cout << "iter_num_:" << GREEN << iter_num_ << RESET << endl;
      cout << "use_node_num_:" << GREEN << use_node_num_ << RESET << endl;
      cout << "search_node_num_:" << GREEN << search_node_num_ << RESET << endl;
      cout << "abandon_node_num_:" << GREEN << abandon_node_num_ << RESET << endl;
      cout << "astarcloselist.size():" << GREEN << astarcloselist.list_length<< RESET << endl;
      cout << "astaropenlist.size():" << GREEN << astaropenlist.list_length << RESET << endl;
      if (astaropenlist.IsAllExtend())
          cout << RED << "aster openlist is all extend" << RESET << endl;
      if (iter_num_ > iter_num_max_)
          cout << RED << "iter_num_ over max" << RESET << endl;
      break;
    }
    //########################################################
    //3.2:get the best node and put it into the close list
    NodeState parNodeState = astaropenlist.OpenListMinPop();
    GridNodePtr parGridNodePtr = parNodeState.CurrGridNodePtr;
    Vector3d parPose = parNodeState.Position;
    Vector3i parnodeindex = parNodeState.CurrGridNodePtr->index;
    astarcloselist.insert(parNodeState.CurrGridNodePtr);
    use_node_num_ ++;

    // ROS_INFO("[\033[32mLazyKinoPRM\033[0m]parPose is (%f,%f,%f)",parPose[0],parPose[1],parPose[2]);

    //########################################################
    if (iter_num_ < DIR_GRID_M)
    DIR_GRID = iter_num_;
    else
    DIR_GRID = DIR_GRID_M;

    for (int dir_row = -DIR_GRID;dir_row <= DIR_GRID;dir_row++)
    {
      if (stop_search_) break;

      for (int dir_col = -DIR_GRID;dir_col <= DIR_GRID;dir_col++)
      {
        if (stop_search_) break;

        for (int dir_dep = 0;dir_dep < MAX_POSE_MAP_D;dir_dep++)
        {
          if (stop_search_) break;
          //自身索引就跳过
          if (dir_row == 0 && dir_col == 0)
              continue;
          //3.3:check if the node is in the close list
          int new_row = parnodeindex(0) + dir_row;
          int new_col = parnodeindex(1) + dir_col;
          int new_dep = dir_dep;
          if (new_row < 0 || new_row >= MAX_POSE_MAP_X ||
              new_col < 0 || new_col >= MAX_POSE_MAP_Y ||
              new_dep < 0 || new_dep >= MAX_POSE_MAP_D)
              //超出pose_map边界,就跳过
              continue;
          if (pose_map[new_row][new_col][new_dep] == NULL)
              {
                ROS_INFO("pose_map[new_row%d][new_col%d][new_dep%d] == NULL",new_row,new_col,new_dep);
                // cout << YELLOW <<"pose_map[new_row][new_col][new_dep] == NULL" << RESET << endl;
                continue;
              }
          search_node_num_ ++;
          GridNodePtr newGridNodePtr = pose_map[new_row][new_col][new_dep];
          newNodeState.clear();
          newNodeState.CurrGridNodePtr = newGridNodePtr;

          //3.4:check if the node is Goal
          if (newGridNodePtr->type == Goal)
          {
            //calculate the new node angle
            Vector3d newPose = newGridNodePtr->pose;
            // init the new node state
            newNodeState.Position = newPose;
            newNodeState.Velocity = VelGoal;
            newNodeState.Acceleration = AccGoal;
            obvpmid.SolveBVP(&parNodeState,&newNodeState);
            goal_cost = getHeuristic(newPose,PointGoal,&newNodeState);
            collision_flag = getPathCost(&parNodeState,&newNodeState);            
            if (collision_flag)
            {
              double _q_ref = angles::shortest_angular_distance(parNodeState.Position[2],newNodeState.Position[2]);
              _q_ref = parNodeState.Position[2] + _q_ref;
              ROS_INFO("Trajectory q = %2.4f",_q_ref);
              no_path_ = false;
              stop_search_ = true;
              goal_node_listindex_ = astaropenlist.list_length;
              astaropenlist.insert(&parNodeState,&newNodeState);
              cout << GREEN << "find a path" << RESET << endl;
              ROS_INFO("Goal Pose is (%2.4f,%2.4f,%2.4f)",newPose[0],newPose[1],newPose[2]);
            }
          }
          if(stop_search_) break;
          //3.5:check if the node is in the close list or is invalid
          if (newGridNodePtr->type == Invalid || astarcloselist.isCloseList(newGridNodePtr))
          {
            abandon_node_num_++;
            continue;
          }
          //3.6: extend the par node
          //calculate the new node angle
          Vector3d newPose = newGridNodePtr->pose;
          newPose[2] = getDeflection(parPose,newPose);
          // init the new node state
          newNodeState.Position = newPose;
          newNodeState.Velocity = Vector3d(0,0,0);
          newNodeState.Acceleration = Vector3d(0,0,0);

          // for code Debug 
          // ROS_INFO("[\033[32mDIR\033[0m]:iter:%d, row=%d,col=%d,dep=%d)",search_node_num_,dir_row,dir_col,dir_dep);
          // ROS_INFO("[\033[32mLazyKinoPRM\033[0m]newPose is (%f,%f,%f)",newPose[0],newPose[1],newPose[2]);
          

          obvpmid.SolveMiniAccInputAcc(&parNodeState,&newNodeState);
          goal_cost = getHeuristic(newPose,PointGoal,&newNodeState);
          collision_flag = getPathCost(&parNodeState,&newNodeState);       
          // ROS_INFO("[\033[32mLazyKinoPRM\033[0m]collision_flag = %d",collision_flag);
          if (collision_flag)
          {
            astaropenlist.insert(&parNodeState,&newNodeState);
            // ROS_INFO("[\033[32mLazyKinoPRM\033[0m]insert a node %d",astaropenlist.list_length);
          }
        }
      }
    }
    /* code */
  }
  //##################################################################################################################
  //step4: get the path
  uint32_t list_idx = goal_node_listindex_;
  NodeState nodestata;
  //clear the path
  pathstateSets.clear();
  for (;list_idx != 0;)
  {
    nodestata = astaropenlist.nodestateSets.at(list_idx);
    pathstateSets.push_back(nodestata);
    list_idx = nodestata.Parenodelistindex;
  }
  //reverse the path
  reverse(pathstateSets.begin(),pathstateSets.end());
  cout << "pathstateSets.size() = " << YELLOW <<  pathstateSets.size() << RESET << endl;
  return no_path_;
}


/**************************************************************************************************
  * @brief: get the node pointer from the index
  * @param: _index: the index
  * @return: the node pointer if out rang is NULL
*/
GridNodePtr LazyKinoPRM::Index2PoseNode(Eigen::Vector3i _index)
{
  GridNodePtr _node = NULL;
  if (_index(0) < 0 || _index(0) >= MAX_POSE_MAP_X || _index(1) < 0 || _index(1) >= MAX_POSE_MAP_Y || _index(2) < 0 || _index(2) >= MAX_POSE_MAP_D)
    return _node;
  else
  {
    _node = pose_map[_index(0)][_index(1)][_index(2)];
    return _node;
  }
}

/**************************************************************************************************
  * @brief: get the node pointer from the pose
  * @param: _pose: the pose
  * @return: the node pointer if out rang is NULL
*/
GridNodePtr LazyKinoPRM::Pose2PoseNode(Eigen::Vector3d _pose)
{
  int _x,_y,_z;
  _x = (int)floor( (_pose(0) + map_origin_[0]) / xy_sample_size_);
  _y = (int)floor( (_pose(1) + map_origin_[1]) / xy_sample_size_);
  // _z = (int)floor( (_pose(2) / axi_resolution_);
  _z= 0;

  GridNodePtr _node = NULL;
  if (_x < 0 || _x >= MAX_POSE_MAP_X || _y < 0 || _y >= MAX_POSE_MAP_Y || _z < 0 || _z >= MAX_POSE_MAP_D)
    return _node;
  else
  {
    _node = pose_map[_x][_y][_z];
    return _node;
  }
}

/**************************************************************************************************
  * @brief: get the index from the pose
  * @param: _pose: the pose
  * @return: the index
*/
Eigen::Vector3i LazyKinoPRM::Pose2Index(Eigen::Vector3d _pose)
{
  Eigen::Vector3i _index;
  _index(0) = (int)floor( (_pose(0) + map_origin_[0]) / xy_sample_size_);
  _index(1) = (int)floor( (_pose(1) + map_origin_[1]) / xy_sample_size_);
  _index(2) = 0;
  // _index(2) = floor(_pose(2) / axi_resolution_);
  return _index;
}

//return the min angle delta 
/**********************************************************************************************************************
  * @brief: get the min angle delta
  * @param: _start: start point
  * @param: _goal: goal point
  * @return: the min angle delta
*/
double LazyKinoPRM::AngleMinDelta(Eigen::Vector3d _start, Eigen::Vector3d _goal)
{
  double _anggoal, _angstart, _delta;
  _anggoal = _goal(2);
  _angstart = _start(2);
  // if (abs(_delta) > )
  _anggoal = fmod(_anggoal, M_PI * 2);
  if (_anggoal < 0)
    _anggoal = _anggoal + M_PI * 2;
  _angstart = fmod(_angstart, M_PI * 2);
  if (_angstart < 0)
    _angstart = _angstart + M_PI * 2;
  _delta = _anggoal - _angstart;
  if (abs(_delta) > M_PI)
  {
    if (_delta > 0)
      _delta = _delta - M_PI * 2;
    else
      _delta = _delta + M_PI * 2;
  }
  return _delta;
}


/**********************************************************************************************************************
  * @brief: get the heuristic cost
  * @param: _start: start point
  * @param: _goal: goal point
  * @return: the heuristic cost
*/
inline double LazyKinoPRM::getHeuristic(Eigen::Vector3d _start, Eigen::Vector3d _goal,NodeStatePtr _nodestate)
{
  double gncost = 0;
  gncost = sqrt(pow(_start(0) - _goal(0), 2) + pow(_start(1) - _goal(1), 2));
  _nodestate->heurcost = gncost;
  return gncost;
}

/**********************************************************************************************************************
  * @brief: get the cost of the path
  * @param: _start: start point
  * @param: _goal: goal point
  * @param: _pathstate: the path state
  * @return: the cost of the path and the path collision flag
*/
inline bool LazyKinoPRM::getPathCost(NodeStatePtr _parnodestate,NodeStatePtr _curnodestate)
{
  Eigen::Vector3d _start, _goal;
  _start = _parnodestate->Position;
  _goal = _curnodestate->Position;
  bool collision_flag = false;
  double pathlength = 0,angcost = 0,angdelta = 0;
  std::vector<double> xtraj,ytraj;
  xtraj = _curnodestate->polytraj('p','x',time_interval_);
  ytraj = _curnodestate->polytraj('p','y',time_interval_);
  collision_flag = TrajectoryCheck(xtraj,ytraj);
  for (int i = 0; i < xtraj.size()-1; i++)
  {
    pathlength += sqrt(pow(xtraj[i] - xtraj[i+1],2) + pow(ytraj[i] - ytraj[i+1],2));
  }
  angdelta = AngleMinDelta(_start,_goal);
  _curnodestate->angledelta = angdelta;
  _curnodestate->distance   = sqrt(pow(_start(0) - _goal(0), 2) + pow(_start(1) - _goal(1), 2));
  angcost = pow(angdelta,2)*c_angle_;
  _curnodestate->trajectory_length = pathlength;
  _curnodestate->angle_cost = angcost;
  //all waypoints path cost
  _curnodestate->pathcost = _parnodestate->pathcost + pathlength + angcost;
  return collision_flag;
}

///* Tag: floor is not suitable */
// inline bool LazyKinoPRM::setObstacleMap(const double coord_x, const double coord_y, const double coord_z)
// {
//   int map_rows = floor((coord_x+map_origin_[0])/xy_resolution_); 
//   int map_cols = floor((coord_y+map_origin_[1])/xy_resolution_);
//   if (map_rows < 0 || map_cols < 0 || map_cols >= MAX_OBS_MAP_COL || map_rows >= MAX_OBS_MAP_ROW)
//     return false;
//   else
//   {
//     obs_map->at<uchar>(map_rows,map_cols) = IMG_OBS;
//     return true;
//   }
// }

inline bool LazyKinoPRM::setObstacleMap(const double coord_x, const double coord_y, const double coord_z)
{
  int map_rows_f = floor((coord_x+map_origin_[0])/xy_resolution_); 
  int map_cols_f = floor((coord_y+map_origin_[1])/xy_resolution_);
  int map_rows_c = ceil((coord_x+map_origin_[0])/xy_resolution_); 
  int map_cols_c = ceil((coord_y+map_origin_[1])/xy_resolution_);

  raw_obs_map->at<uchar>(cv::Point(map_rows_f, map_cols_f)) = IMG_OBS;
  raw_obs_map->at<uchar>(cv::Point(map_rows_c, map_cols_c)) = IMG_OBS;
  raw_obs_map->at<uchar>(cv::Point(map_rows_f, map_cols_c)) = IMG_OBS;
  raw_obs_map->at<uchar>(cv::Point(map_rows_c, map_cols_f)) = IMG_OBS;

  if (map_rows_f < 0 || map_cols_f < 0 || map_cols_f >= MAX_OBS_MAP_COL || map_rows_f >= MAX_OBS_MAP_ROW\
      || map_rows_c < 0 || map_cols_c < 0 || map_cols_c >= MAX_OBS_MAP_COL || map_rows_c >= MAX_OBS_MAP_ROW)
    return false;
  else
  {
    return true;
  }
}

/**********************************************************************************************************************
 * @description:  check pose is obsticle feasible or not,and out of map is not free
 * @reference: 
 * @param {Eigen::Vector3d} _pose
 * @return {bool} feasible  no -> false; yes -> true
 */
inline bool LazyKinoPRM::isObstacleFree(Eigen::Vector3d _pose)
{
  bool feasible=true;
  ////real pose in map
    int map_rows = floor((_pose[0]+map_origin_[0])/xy_resolution_); 
    int map_cols = floor((_pose[1]+map_origin_[1])/xy_resolution_);
    //out map or is not free
    if ( map_rows < 0 || map_cols < 0 || map_cols >= MAX_OBS_MAP_COL || map_rows >= MAX_OBS_MAP_ROW
        || obs_map->at<uchar>(cv::Point(map_rows,map_cols)) == IMG_OBS)
      {feasible=false;return feasible;}
    return feasible;
}


/******
 * @description:  set LazyKinoPRM param
 * @reference:  
 * @param {NodeHandle&} nh
 * @return {*}
 *///zwt
void LazyKinoPRM::setParam(ros::NodeHandle& nh)
{
  //#######################################################
  // param init for planner
  double map_size_x, map_size_y, map_size_z;
  double map_origin_x, map_origin_y, map_origin_z;
  double dir_grid_m_=2;
  nh.param("search/dir_grid_m", dir_grid_m_, 3.0);
  nh.param("search/iter_num_max", iter_num_max_,600);
  nh.param("search/xy_sample_size", xy_sample_size_, 0.4);
  nh.param("search/q_sample_size", q_sample_size_, M_PI*2);
  nh.param("search/xy_bais", xy_bais_, 0.2);
  nh.param("search/c_angle", c_angle_, 0.2);
  nh.param("search/xy_resolution", xy_resolution_, 0.05);
  nh.param("search/q_resolution", axi_resolution_, M_PI*2);
  nh.param("search/time_interval", time_interval_, 0.01);
  nh.param("search/vel_factor", vel_factor_, 2.0);
  nh.param("search/ome_factor", ome_factor_, 1.0);
  nh.param("search/weightR", weightR_, 1.0);
  nh.param("search/SAMPLE_RANDOM", SAMPLE_RANDOM, true);

  nh.param("map/x_size",  map_size_x, 50.0);
  nh.param("map/y_size",  map_size_y, 50.0);
  nh.param("map/z_size",  map_size_z, 5.0 );
  nh.param("map/erode_kernel_size",  erode_kernel_size_, int(3.0));

  // nh.param("map/orign_x_size",  map_origin_x, 25.0);
  // nh.param("map/orign_y_size",  map_origin_y, 25.0);
  // nh.param("map/orign_z_size",  map_origin_z, 2.5 );
  
  //#######################################################
  // param init without param server
  // dir_grid_m_ = 3.0;
  // iter_num_max_ = 600;
  // xy_sample_size_ = 0.4;
  // q_sample_size_  = M_PI*2;
  // xy_bais_ = xy_sample_size_ / 2.0;
  // xy_resolution_ = 0.05;
  // axi_resolution_ = M_PI*2;
  // time_interval_ = 0.01;
  // vel_factor_ = 2;
  // ome_factor_ = 1;
  // SAMPLE_RANDOM = true;
  // map_size_x = 50.0;
  // map_size_y = 50.0;
  // map_size_z = 5.0;
  // map_origin_x = 25.0;
  // map_origin_y = 25.0;
  // map_origin_z = 2.5;
  //#######################################################
  // param calculate
  // c_angle_ = xy_sample_size_ /M_PI * 1.2;
  DIR_GRID_M = (uint16_t)abs(dir_grid_m_);
  // map_origin_[0] = map_origin_x;
  // map_origin_[1] = map_origin_y;
  // map_origin_[2] = map_origin_z;
  map_size_[0] = map_size_x;
  map_size_[1] = map_size_y;
  map_size_[2] = map_size_z;
  map_origin_[0] = map_size_x/2;
  map_origin_[1] = map_size_y/2;
  map_origin_[2] = map_size_z/2;
  IMG_OBS = 0;// scale 0-255 0=black 255=white
}


/**********************************************************************************************************************
  * @description:  init the lazyKinoPRM
  * @reference:  
  * @param {*}
  * @return {*}
*/
void LazyKinoPRM::init()
{
  /* ---------- map params ---------- */
  // init the map
  MAX_OBS_MAP_ROW = (uint16_t)ceil(map_size_[0] / xy_resolution_); //for x
  MAX_OBS_MAP_COL = (uint16_t)ceil(map_size_[1] / xy_resolution_); //for y
  MAX_POSE_MAP_X  = (uint16_t)ceil(map_size_[0] / xy_sample_size_); //for x
  MAX_POSE_MAP_Y  = (uint16_t)ceil(map_size_[1] / xy_sample_size_); //for y
  MAX_POSE_MAP_D  = (uint16_t)ceil(2 * M_PI / q_sample_size_); //for q
  //init obs map as all free 255 = white
  // in opencv Mat :row == heigh == Point.y;col == width == Point.x;Mat::at(Point(x, y)) == Mat::at(y,x)
  obs_map = new cv::Mat(MAX_OBS_MAP_COL, MAX_OBS_MAP_ROW, CV_8UC1, cv::Scalar(255));
  sdf_map = new cv::Mat(MAX_OBS_MAP_COL, MAX_OBS_MAP_ROW, CV_32FC1, cv::Scalar(255));
  raw_obs_map = new cv::Mat(MAX_OBS_MAP_COL, MAX_OBS_MAP_ROW, CV_8UC1, cv::Scalar(255));
  element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(erode_kernel_size_, erode_kernel_size_));
  //init pose map as all free
  /*// Tag: the 3D point GridNodePtr maybe wrong;*/
  // Allocate memory for the pose map.
  pose_map = new GridNodePtr **[MAX_POSE_MAP_X];
  

#ifdef ENABLE_DEBUFG_FLAG
  ROS_DEBUG("obs_map size: %d, %d", MAX_OBS_MAP_ROW, MAX_OBS_MAP_COL);
  ROS_DEBUG("pose_map size: %d, %d, %d", MAX_POSE_MAP_X, MAX_POSE_MAP_Y, MAX_POSE_MAP_D);
#endif

  if(pose_map == nullptr)
    ROS_ERROR("pose_map is NULL");
    // else
    // ROS_INFO("pose_map is not NULL");
  assert(pose_map != nullptr);

   for (uint32_t map_rows = 0; map_rows < MAX_POSE_MAP_X; map_rows++)
  {
    pose_map[map_rows] = new GridNodePtr *[MAX_POSE_MAP_Y];
    if (pose_map == nullptr)
    ROS_ERROR("pose_map[%d] is NULL",map_rows);
    assert(pose_map != nullptr);

    // ROS_DEBUG("pose_map[%d] is not NULL", map_rows);

    for (uint32_t map_cols = 0; map_cols < MAX_POSE_MAP_Y; map_cols++)
    {
      pose_map[map_rows][map_cols] = new GridNodePtr[MAX_POSE_MAP_D];
      if(pose_map[map_rows][map_cols] == nullptr)
        ROS_ERROR("pose_map[%d][%d] is NULL", map_rows, map_cols);
      assert(pose_map[map_rows][map_cols] != nullptr);

      // ROS_DEBUG("pose_map[%d][%d] is not NULL", map_rows, map_cols);

      for (uint32_t map_depth = 0; map_depth < MAX_POSE_MAP_D; map_depth++)
      {
        pose_map[map_rows][map_cols][map_depth] = new GridNode;
        if(pose_map[map_rows][map_cols][map_depth] == NULL)
          ROS_ERROR("pose_map[%d][%d][%d] is NULL", map_cols, map_rows, map_depth);
        assert(pose_map[map_rows][map_cols][map_depth] != nullptr);

        // ROS_DEBUG("pose_map[%d][%d][%d] is not NULL", map_cols, map_rows, map_depth);

        Eigen::Vector3d rand_pose;
        Eigen::Vector3i rand_pose_index(map_rows, map_cols, map_depth);
        double pose_x, pose_y;
        if (SAMPLE_RANDOM)
        {
          pose_y = map_cols * xy_sample_size_ + ((double)rand() / RAND_MAX - 0.5) * xy_bais_ + xy_sample_size_ / 2 - map_origin_[1];
          pose_x = map_rows * xy_sample_size_ + ((double)rand() / RAND_MAX - 0.5) * xy_bais_ + xy_sample_size_ / 2 - map_origin_[0];
        }
        else
        {
          pose_y = map_cols * xy_sample_size_ + map_depth * xy_bais_ - map_origin_[1];
          pose_x = map_rows * xy_sample_size_ + map_depth * xy_bais_ - map_origin_[0];
        }
        //// sample yaw angle (not used), so depth is also sampled in xy plane
        // IMG_OBS is white in the image there is "0";
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        rand_pose[0] = pose_x;
        rand_pose[1] = pose_y;
        pose_map[map_rows][map_cols][map_depth]->setPose(rand_pose, rand_pose_index);
        if (isObstacleFree(rand_pose))
          // free space is Type=M
          pose_map[map_rows][map_cols][map_depth]->setType(Mid);
        else
          // occupied space is Type=W
          pose_map[map_rows][map_cols][map_depth]->setType(Invalid);
      }// end  map_depth
    }// end map_cols
  }// end  map_rows
  ///* reset the search parametres */
  iter_num_ = 0;
  no_path_ = true;
  stop_search_ = false;
  extendflag_ = false;
  use_node_num_ = 0;
  //reset the search 还未使用vector就释放,会出现问题,造成内存重复释放
  // astaropenlist.reset();
  // astarcloselist.reset();
}// 均匀空间撒点

/**********************************************************************************************************
  * @brief: sample in the map
  * @param: none
  * @return: none
*/
void LazyKinoPRM::sample()
{
  // reset the pose map all free node sample pose and set as mid type
  for (uint32_t map_rows = 0; map_rows < MAX_POSE_MAP_X; map_rows++)
  {
    for (uint32_t map_cols = 0; map_cols < MAX_POSE_MAP_Y; map_cols++)
    {
      for (uint32_t map_depth = 0; map_depth < MAX_POSE_MAP_D; map_depth++)
      {
        Eigen::Vector3d rand_pose;
        Eigen::Vector3i rand_pose_index(map_rows, map_cols, map_depth);
        double pose_x, pose_y;
        if (SAMPLE_RANDOM)
        {
          pose_y = map_cols * xy_sample_size_ + ((double)rand() / RAND_MAX - 0.5) * xy_bais_ + xy_sample_size_ / 2 - map_origin_[1];
          pose_x = map_rows * xy_sample_size_ + ((double)rand() / RAND_MAX - 0.5) * xy_bais_ + xy_sample_size_ / 2 - map_origin_[0];
        }
        else
        {
          pose_y = map_cols * xy_sample_size_ + map_depth * xy_bais_ - map_origin_[1];
          pose_x = map_rows * xy_sample_size_ + map_depth * xy_bais_ - map_origin_[0];
        }
        rand_pose[0] = pose_x;
        rand_pose[1] = pose_y;
        pose_map[map_rows][map_cols][map_depth]->setPose(rand_pose, rand_pose_index);
        if (isObstacleFree(rand_pose))
          pose_map[map_rows][map_cols][map_depth]->setType(Mid);
        else
          pose_map[map_rows][map_cols][map_depth]->setType(Invalid);
      }
    }
  }
}

/**********************************************************************************************************
  * @brief: reset sample which has been used
  * @param: none
  * @return: none
*/
void LazyKinoPRM::resetsample()
{
  uint32_t map_rows, map_cols, map_depth;
  Eigen::Vector3d rand_pose;
  double pose_x, pose_y;
  Eigen::Vector3i rand_pose_index;
  for (uint32_t idx = 0; idx < int(pathstateSets.size()); idx++)
  {
    map_rows = pathstateSets[idx].CurrGridNodePtr->index[0];
    map_cols = pathstateSets[idx].CurrGridNodePtr->index[1];
    map_depth = pathstateSets[idx].CurrGridNodePtr->index[2];
    rand_pose_index << map_rows, map_cols, map_depth;
    if (SAMPLE_RANDOM)
    {
      pose_y = map_cols * xy_sample_size_ + ((double)rand() / RAND_MAX - 0.5) * xy_bais_ + xy_sample_size_ / 2 - map_origin_[1];
      pose_x = map_rows * xy_sample_size_ + ((double)rand() / RAND_MAX - 0.5) * xy_bais_ + xy_sample_size_ / 2 - map_origin_[0];
    }
    else
    {
      pose_y = map_cols * xy_sample_size_ + map_depth * xy_bais_ - map_origin_[1];
      pose_x = map_rows * xy_sample_size_ + map_depth * xy_bais_ - map_origin_[0];
    }
    rand_pose[0] = pose_x;
    rand_pose[1] = pose_y;
    pose_map[map_rows][map_cols][map_depth]->setPose(rand_pose, rand_pose_index);
    if (isObstacleFree(rand_pose))
      pose_map[map_rows][map_cols][map_depth]->setType(Mid);
    else
      pose_map[map_rows][map_cols][map_depth]->setType(Invalid);
  }
}

/*
  * @brief: reset the pose map of EXTEND,GOAL, START => MID
  * @param: none
  * @return: none
*/
void LazyKinoPRM::reset()
{
  uint32_t map_rows, map_cols, map_depth;
  Eigen::Vector3d rand_pose;
  double pose_x, pose_y;
  Eigen::Vector3i rand_pose_index;
  for (uint32_t idx = 0; idx < int(astaropenlist.nodestateSets.size()); idx++)
  {
    GridNodePtr gridnodeptr = astaropenlist.nodestateSets[idx].CurrGridNodePtr;
    gridnodeptr->setType(Mid);
  }
  //############################################################################################################
  //reset search parameters
  iter_num_ = 0;
  no_path_ = true;
  stop_search_ = false;
  extendflag_ = false;
  use_node_num_ = 0;
  //reset the search 
  astaropenlist.reset();
  astarcloselist.reset();
}

/*
  * @description:  update the obstacle map
  * @reference: 
  * @param {pcl::PointCloud<pcl::PointXYZ>} cloud
  * @return {*}
*/
void LazyKinoPRM::updataObsMap(pcl::PointCloud<pcl::PointXYZ> cloud)
{
  if ((int)cloud.points.size() == 0)
    return;
  pcl::PointXYZ pt;
  for (int idx = 0; idx < (int)cloud.points.size(); idx++)
  {
    pt = cloud.points[idx];
    setObstacleMap(pt.x, pt.y, pt.z);
    // set obstalces into grid map for path planning
    // if (!setObstacleMap(pt.x, pt.y, pt.z))
    //   cout << RED << "set obstacle map failed!, PCL out map range!" << RESET << endl;
  }
  // Tag: local map update? and 
  cv::erode(*raw_obs_map,*obs_map,element);
  cv::Mat binary_map;
  cv::threshold(*obs_map, binary_map, 127, 255, THRESH_BINARY);//二值化阈值处理
  // binary_map = ~binary_map;
  // imshow("binary_map",binary_map);
  // *sdf_map = binary_map.clone();
  cv::Mat sdf_img;
  cv::distanceTransform(binary_map,sdf_img,DIST_L2,DIST_MASK_3);
  // 1 pixel = xy_resolution_ m
  sdf_img = sdf_img * xy_resolution_;
  *sdf_map = sdf_img;
  ROS_DEBUG("update sdf map size: %d, %d",sdf_map->rows,sdf_map->cols);
}

cv::Mat* LazyKinoPRM::getObsMap()
{
  return obs_map;
}

cv::Mat* LazyKinoPRM::getSdfMap()
{
  return sdf_map;
}

/**********************************************************************************************************************
 * @description:  check trajectory is obsticle feasible or not
 * @reference: 
 * @param {vector<double>} xtraj
 * @param {vector<double>} ytraj
 * @return {bool} feasible  yes -> true; no -> false
 */
bool LazyKinoPRM::TrajectoryCheck(vector<double> xtraj,vector<double> ytraj)
{
  //输入为xy轨迹和地图信息，若轨迹线不会经过障碍物，则返回true, 碰到障碍物则为返回false
  bool feasible=true;
  Eigen::Vector3d pose;
  for (int idx=0;idx < xtraj.size();idx++)
  {
    pose[0]=xtraj[idx];
    pose[1]=ytraj[idx];
    feasible=isObstacleFree(pose);
    if (feasible==false)
      return feasible;
  }
  return feasible;
}
