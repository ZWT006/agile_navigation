/*
 * @Author: wentao zhang && zwt190315@163.com
 * @Date: 2023-04-03
 * @LastEditTime: 2023-06-15
 * @Description: 
 * @reference: 
 * 
 * Copyright (c) 2023 by wentao zhang, All Rights Reserved. 
 */

#ifndef _GRIDNODE_H_
#define _GRIDNODE_H_

#include <vector>
#include <iostream>
#include <Eigen/Eigen>

#include "Polynomial.h"

//the following are UBUNTU/LINUX ONLY terminal color codes.
#define RESET   "\033[0m"       /* Reset */
#define BLACK   "\033[30m"      /* Black */
#define RED     "\033[31m"      /* Red */
#define GREEN   "\033[32m"      /* Green */
#define YELLOW  "\033[33m"      /* Yellow */
#define BLUE    "\033[34m"      /* Blue */
#define MAGENTA "\033[35m"      /* Magenta */
#define CYAN    "\033[36m"      /* Cyan */
#define WHITE   "\033[37m"      /* White */
#define BOLDBLACK   "\033[1m\033[30m"      /* Bold Black */
#define BOLDRED     "\033[1m\033[31m"      /* Bold Red */
#define BOLDGREEN   "\033[1m\033[32m"      /* Bold Green */
#define BOLDYELLOW  "\033[1m\033[33m"      /* Bold Yellow */
#define BOLDBLUE    "\033[1m\033[34m"      /* Bold Blue */
#define BOLDMAGENTA "\033[1m\033[35m"      /* Bold Magenta */
#define BOLDCYAN    "\033[1m\033[36m"      /* Bold Cyan */
#define BOLDWHITE   "\033[1m\033[37m"      /* Bold White */

#define Naf 9999

// typedef struct Score 
// {
  
// }ScoreStc;
// ScoreStc NodeScore;

//############################################################################
//node class
//Node Type list:Start -> S;Goal -> G;Midpoint -> M;Invalid -> W;Extend -> E;
enum GridNodeType_Set {Start,Goal,Mid,Invalid,Extend};
struct GridNode;
typedef GridNode* GridNodePtr;

struct GridNode
{
// private:
  GridNodeType_Set type;             //  point type:
  Eigen::Vector3d pose;  // pose of gridnode  [x,y,θ]
  Eigen::Vector3d posi;  // position
  Eigen::Vector3d orit;  // oritention
  // Eigen::Vector3i dir;   // direction of expanding
  Eigen::Vector3i index; // pose_map index [rows,cols,deepth]
  // double gScore, fScore;
  // NodeScore
  // GridNodePtr parentPtr;
  // std::multimap<double, GridNodePtr>::iterator nodeMapIt;

  // GridNode(Eigen::Vector3d _Pose, Eigen::Vector3i _Index,GridNodeType_Set _Type)
  // {
  //   type = _Type;
  //   pose = _Pose;
  //   index = _Index;
  //   // type = Invalid;
  //   // pose = Eigen::Vector3d(0,0,0);
  //   // index = Eigen::Vector3i(0,0,0);
  //   // parentPtr = NULL;
    
  //   // gScore = Naf;
  //   // fScore = Naf;
    
  // }
// public:
  // GridNode(Eigen::Vector3d _Pose, Eigen::Vector3i _Index, GridNodeType_Set _Type);
  GridNode(){};
  ~GridNode(){};
  void setPose(Eigen::Vector3d _Pose, Eigen::Vector3i _Index);
  void setType(GridNodeType_Set _Type);
  // // set Pose function
  // void setPose(Eigen::Vector3d _Pose, Eigen::Vector3i _Index)
  // {
  //   pose = _Pose;
  //   index = _Index;
  // }
  // // set Type function
  // void setType(GridNodeType_Set _Type)
  // {
  //   type = _Type;
  // }
};

//############################################################################
//node state structure
// Eigen::Vector3d Position;
// Eigen::Vector3d Velocity;
// Eigen::Vector3d Acceleration;
// double Trajectory_Cost;
// double Trajectory_Length;
// bool collision_check; // False -> no collision, True -> collision
// bool optimal_flag;
struct NodeState;
typedef NodeState* NodeStatePtr;

struct NodeState
{
    Eigen::Vector3d Position;
    Eigen::Vector3d Velocity;
    Eigen::Vector3d Acceleration;
    

    Eigen::Vector3d ParePosition;
    Eigen::Vector3d PareVelocity;
    Eigen::Vector3d PareAcceleration;

    uint32_t Currnodelistindex;
    uint32_t Parenodelistindex;

    GridNodePtr CurrGridNodePtr;
    GridNodePtr PareGridNodePtr;
    
    double referT;
    double dt;
    Polynomial poly;
    double distance,angledelta;
    double angle_cost;        //angle delta cost
    double trajectory_length; //tarjectory length cost
    double trajectory_cost;   //tarjectory cost by dynamic model
    double fncost,heurcost,pathcost;  // fncost = heuristiccost + pathcost; pathcost = angle_cost + trajectory_length + parnodestate.pathcost;
    bool collision_check = false; // True -> no collision, False -> collision
    bool dynamic_flag;    // False -> not optimal in TraLibrary, True -> not optimal in TraLibrary,

    std::vector<double> xpcoeff; //x polynomial coeffs
    std::vector<double> ypcoeff; //y polynomial coeffs
    std::vector<double> qpcoeff; //q polynomial coeffs

    std::vector<double> xvcoeff; //x polynomial coeffs
    std::vector<double> yvcoeff; //y polynomial coeffs
    std::vector<double> qvcoeff; //q polynomial coeffs

    std::vector<double> xacoeff; //x polynomial coeffs
    std::vector<double> yacoeff; //y polynomial coeffs
    std::vector<double> qacoeff; //q polynomial coeffs

    std::vector<double> xptraj; //xptraj
    std::vector<double> yptraj; //yptraj
    std::vector<double> qptraj; //qptraj
    
    std::vector<double> xvtraj; //xvtraj
    std::vector<double> yvtraj; //yvtraj
    std::vector<double> qvtraj; //qvtraj
    // NodeState(Eigen::Vector3d _Position, Eigen::Vector3d _Velocity, Eigen::Vector3d _Acceleration, double _Trajectory_Cost,double _Trajectory_Length)
    // {
    //   Position = _Position;
    //   Velocity = _Velocity;
    //   Acceleration = _Acceleration;
    //   Trajectory_Cost = _Trajectory_Cost;
    //   Trajectory_Length = _Trajectory_Length;
    //   collision_check = false;
    //   optimal_flag = false;
    // }
    NodeState(){};
    ~NodeState(){};
    void setCollisioncheck(bool _collision_check);
    void setSelfPose(Eigen::Vector3d _Position);

    void setSelfNodeState(Eigen::Vector3d _Position, Eigen::Vector3d _Velocity, Eigen::Vector3d _Acceleration);
    void setParentNodeState(Eigen::Vector3d _Position, Eigen::Vector3d _Velocity, Eigen::Vector3d _Acceleration);
    std::vector<double> polytraj(char _type,char _dimen,double _dt);
    // double polytrajdot(char _type,char _dimen,double _dt);
    double polytrajStart(char _type, char _dimen);
    double polytrajEnd(char _type, char _dimen);
    void pushcoeff(std::vector<double> _xcoeff,std::vector<double> _ycoeff,std::vector<double> _qcoeff);
    void pushGridNode(GridNodePtr _CurrGridNodePtr);
    void clear();
    // void setCollisionfree()
    // {
    //   collision_check = true;
    // }
    // void setOptimal()
    // {
    //   optimal_flag = true;
    // }
};

//############################################################################
//TODO: remove this structure
//node state structure
struct PathState;
typedef PathState* PathStatePtr;
struct PathState
{
  Eigen::Vector3d spi;
  Eigen::Vector3d svi;
  Eigen::Vector3d sai;
  Eigen::Vector3d spf;
  Eigen::Vector3d svf;
  Eigen::Vector3d saf;
  double referT;
  double dt;
  double costJ;
  double trajlength;
  Polynomial poly;

  std::vector<double> xpcoeff; //x polynomial coeffs
  std::vector<double> ypcoeff; //y polynomial coeffs
  std::vector<double> qpcoeff; //q polynomial coeffs

  std::vector<double> xvcoeff; //x polynomial coeffs
  std::vector<double> yvcoeff; //y polynomial coeffs
  std::vector<double> qvcoeff; //q polynomial coeffs

  std::vector<double> xacoeff; //x polynomial coeffs
  std::vector<double> yacoeff; //y polynomial coeffs
  std::vector<double> qacoeff; //q polynomial coeffs

  std::vector<double> xptraj; //xptraj 
  std::vector<double> yptraj; //yptraj
  std::vector<double> qptraj; //qptraj

  std::vector<double> xvtraj; //xvtraj 
  std::vector<double> yvtraj; //yvtraj
  std::vector<double> qvtraj; //qvtraj

  std::vector<double> xatraj; //xatraj 
  std::vector<double> yatraj; //yatraj
  std::vector<double> qatraj; //qatraj

  //######################################################
  PathState(){};
    ~PathState(){};
  
  // _type = 'p' -> position; 'v' -> velocity; 'a' -> acceleration
  // _dimen = 'x' -> x; 'y' -> y; 'q' -> q
  // _dt = time interval
  std::vector<double> polytraj(char _type,char _dimen,double _dt);
  // std::vector<double> xtraj();
  // std::vector<double> ytraj();
  // std::vector<double> qtraj();
  // std::vector<double> xtraj()
  // {
  //   std::vector<double> traj;
  //   std::vector<double> timeseq;
  //   timeseq = poly.getseqence(0.0,referT,dt);
  //   traj = poly.PolyvalVector(xcoeff,timeseq);
  //   return traj;
  // }
  // std::vector<double> ytraj()
  // {
  //   std::vector<double> traj;
  //   std::vector<double> timeseq;
  //   timeseq = poly.getseqence(0.0,referT,dt);
  //   traj = poly.PolyvalVector(ycoeff,timeseq);
  //   return traj;
  // }
  // std::vector<double> qtraj()
  // {
  //   std::vector<double> traj;
  //   std::vector<double> timeseq;
  //   timeseq = poly.getseqence(0.0,referT,dt);
  //   traj = poly.PolyvalVector(ycoeff,timeseq);
  //   return traj;
  // }
};

#endif //_GRIDNODE_H_