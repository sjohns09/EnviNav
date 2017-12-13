/*
 * RRTPlanner.h
 *
 *  Created on: Dec 9, 2017
 *      Author: sammie
 *
 *      BSD License
 */
#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <angles/angles.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>

#ifndef ENVI_NAV_SRC_RRTPLANNER_H_
#define ENVI_NAV_SRC_RRTPLANNER_H_

class RRTPlanner : public nav_core::BaseGlobalPlanner {
 public:
  RRTPlanner();
  RRTPlanner(std::string name, costmap_2d::Costmap2DROS* costmapRos);
  void initialize(std::string name, costmap_2d::Costmap2DROS* costmapRos);
  void initialize(std::string name, int mapSizeX, int mapSizeY, float resolution, float originX,
                  float originY, costmap_2d::Costmap2DROS map);
  bool makePlan(const geometry_msgs::PoseStamped& start,
                const geometry_msgs::PoseStamped& goal,
                std::vector<geometry_msgs::PoseStamped>& plan);
  virtual ~RRTPlanner();

 private:
  struct qTree {
    geometry_msgs::PoseStamped q;
    geometry_msgs::PoseStamped qNear;
    int myIndex;
    int nearIndex;
  };

  geometry_msgs::PoseStamped rand_config();
  int nearest_vertex(geometry_msgs::PoseStamped qRand);
  bool path_safe(geometry_msgs::PoseStamped qRand, int iNear);
  bool check_goal(geometry_msgs::PoseStamped qNew, int iNew);
  bool build_plan();
  void rviz_map(double& x, double& y);
  void map_rviz(double& x, double& y);

  std::vector<qTree> _treeGraph;
  std::vector<geometry_msgs::PoseStamped> _plan;
  geometry_msgs::PoseStamped _goal;
  geometry_msgs::PoseStamped _start;
  costmap_2d::Costmap2DROS* _costmapROS;
  costmap_2d::Costmap2D* _costmap;
  float _resolution, _originX, _originY;
  int _mapSizeX, _mapSizeY;
  bool _initialized;

};

#endif /* ENVI_NAV_SRC_RRTPLANNER_H_ */
