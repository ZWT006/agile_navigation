#ifndef _KINODYNAMIC_ASTAR_H
#define _KINODYNAMIC_ASTAR_H


#include <Eigen/Eigen>
#include <boost/functional/hash.hpp>
#include <iostream>
#include <map>
#include <queue>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <opencv2/opencv.hpp>
#include "LazyPRM.h"
#include "SampleOBVP.h"
#include "GridNode.h"


class LazyKinoPRM {
 private:
  /* ---------- main data structure ---------- */
  GridNodePtr ***  pose_map;  // pose map for gridnode
  // OpenList astaropenlist;            // open list for astar search
  CloseList astarcloselist;          // close list for astar search
  bool stop_search_ = false;  // stop search flag
  bool no_path_ = true;      // no path flag
  bool extendflag_ = false;   // extend flag for lazy prm
  
  /* ---------- record data ---------- */
  Eigen::Vector3d PointStart, PointGoal,VelStart,VelGoal,AccStart,AccGoal;  // start and goal pose
  Eigen::Vector3i start_pose_index_, goal_pose_index_;

  /* ---------- parameter ---------- */
  /* search */
  uint16_t DIR_GRID; //extend grid size
  uint16_t DIR_GRID_M;  //max extend grid size
  uint16_t MAX_POSE_MAP_X,MAX_POSE_MAP_Y,MAX_POSE_MAP_D; //pose map size
  double xy_sample_size_,q_sample_size_;  // sampling sample size
  double xy_bais_;       // sampling bais
  double c_angle_; //angle cost weight
  bool SAMPLE_RANDOM; //sample random or not
  double time_interval_; //time interval for dynamic model
  double vel_factor_; //velocity factor for dynamic model
  double ome_factor_; //angular velocity factor for dynamic model
  double weightR_; //weight for R as angle obvp cost
  
  /* map for obsticle*/
  
  // fixd as params set in launch file
  uint32_t MAX_OBS_MAP_COL,MAX_OBS_MAP_ROW;
  Eigen::Vector3d map_origin_,map_size_;
  uint16_t IMG_OBS;
  uint16_t IMG_FREE;
  int erode_pcl_size_; // raw pcl erode size
  int erode_obs_size_; // obs map erode size
  double sdf_th_;

  /* search mid datas */
  int32_t iter_num_, iter_num_max_,use_node_num_,abandon_node_num_,search_node_num_;       // iteration number
  int32_t dir_row,dir_col,dir_deep; //extend grid dir
  int32_t goal_node_listindex_;
  double goal_cost,path_cost,angle_cost,traj_cost;
  bool collision_flag;

  /* helper function */
  GridNodePtr Index2PoseNode(Eigen::Vector3i _index);
  GridNodePtr Pose2PoseNode(Eigen::Vector3d _pose);
  Eigen::Vector3i Pose2Index(Eigen::Vector3d _pose);
  double AngleMinDelta(Eigen::Vector3d _start,Eigen::Vector3d _goal);
  double getDeflection(Eigen::Vector3d _start,Eigen::Vector3d _goal)
  {
    double angle; // angle = atan2(y,x)
    angle = atan2((_goal(1)-_start(1)),(_goal(0)-_start(0)));
    return angle;
  };
  
  /* search function */
  double getHeuristic(Eigen::Vector3d _start, Eigen::Vector3d _goal,NodeStatePtr _nodestate);
  bool getPathCost(NodeStatePtr _parnodestate,NodeStatePtr _curnodestate);
  bool isObstacleFree(Eigen::Vector3d _pose);
  bool isWideObstacleFree(Eigen::Vector3d _pose);
  bool isFatObstacleFree(Eigen::Vector3d _pose);
  float getPoseSDF(Eigen::Vector3d _pose);
  bool setObstacleMap(const double coord_x, const double coord_y, const double coord_z);
  double AngleCost(Eigen::Vector3d _start,Eigen::Vector3d _goal)
  {
    double angle,cost;
    angle = AngleMinDelta(_start,_goal);
    cost = c_angle_ * angle * angle ;
    return cost;
  };
//   Eigen::Vector3d GoalFeasibleSet(Eigen::Vector3d _goal);
  bool TrajectoryCheck(std::vector<double> *xtraj,std::vector<double> *ytraj);
  
  public:
  /*key datas*/
  cv::Mat *raw_pcl_map;        // pcl map as cv::Mat one pixel is pcl map resolution
  cv::Mat *obs_map;            // occupancy map as cv::Mat one pixel is pcl map resolution
  cv::Mat *fat_map;            // pcl map as cv::Mat one pixel is pcl map resolution
  cv::Mat *sdf_map;            // sdf map as cv::Mat one pixel is pcl map resolution
  cv::Mat pcl_element;            // element for dilate and erode
  cv::Mat obs_element;            // element for dilate and erode
  double xy_resolution_,axi_resolution_;
  std::vector<NodeState> pathstateSets;    // path state list
  OpenList astaropenlist;            // open list for astar search
  Eigen::Vector3d _goal_pose;  // goal pose
  LazyKinoPRM(){};
  ~LazyKinoPRM();

  /* main API */
  void setParam(ros::NodeHandle& nh);
  void init();
  void sample();
  void resetsample();
  void reset();
  bool search(Eigen::Vector3d start_pos, Eigen::Vector3d start_vel,
             Eigen::Vector3d start_acc, Eigen::Vector3d goal_pos,
             Eigen::Vector3d goal_vel,  Eigen::Vector3d goal_acc);
  bool resetLocalMap(Eigen::Vector3d _pose,double radius);
  void updataObsMap(pcl::PointCloud<pcl::PointXYZ> &cloud);
  int getSampleNum();
  GridNodePtr getNodePtr(int index);
  cv::Mat* getObsMap();
  cv::Mat* getSdfMap();
  cv::Mat* getFatMap();
  

//check Trajectory is obstacle feasible or not
  
  bool LongTrajCheck(const std::vector<double> *xtraj,const std::vector<double> *ytraj,int *obs_index);
//   bool PathNodeCheck(std::vector<Eigen::Vector3d> *nodes,int *obs_index);
//   int PathStateSetsCheck(int path_index);


  // typedef shared_ptr<LazyKinoPRM> Ptr;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


#endif