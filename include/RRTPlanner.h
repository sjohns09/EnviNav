/*
 * RRTPlanner.h
 *
 *  Created on: Dec 9, 2017
 *      Author: sammie
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
  void initialize (std::string name, costmap_2d::Costmap2DROS* costmapRos);
  bool makePlan(const geometry_msgs::PoseStamped& start,
                const geometry_msgs::PoseStamped& goal,
                std::vector<geometry_msgs::PoseStamped>& plan);
  virtual ~RRTPlanner();

 private:
  bool is_robot_safe();
  costmap_2d::Costmap2DROS* _costmapROS;
  costmap_2d::Costmap2D* _costmap;
  base_local_planner::WorldModel* _localModel;

};

#endif /* ENVI_NAV_SRC_RRTPLANNER_H_ */
