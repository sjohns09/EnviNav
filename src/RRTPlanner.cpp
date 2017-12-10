/*
 * RRTPlanner.cpp
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
#include "RRTPlanner.h"
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(RRTPlanner, nav_core::BaseGlobalPlanner)

RRTPlanner::RRTPlanner() {
  ROS_ERROR("Need to call constructor with name and costmap");
  //_costmap(NULL);
  //_localModel = NULL;
}

RRTPlanner::RRTPlanner(std::string name, costmap_2d::Costmap2DROS* costmapRos) {
  initialize(name, costmapRos);
}

void RRTPlanner::initialize(std::string name,
                            costmap_2d::Costmap2DROS* costmapRos) {
  _costmapROS = costmapRos;
  _costmap = costmapRos->getCostmap();

  //_localModel = new base_local_planner::CostmapModel(*_costmap);

  ROS_INFO("RRT Planner initialized!!");

}

bool RRTPlanner::is_robot_safe() {
  std::vector<geometry_msgs::Point> roboBasePrint = _costmapROS->getRobotFootprint();

  return false;
}

bool RRTPlanner::makePlan(const geometry_msgs::PoseStamped& start,
                          const geometry_msgs::PoseStamped& goal,
                          std::vector<geometry_msgs::PoseStamped>& plan) {

  ROS_INFO("Making Plan!");

  return true;

}

RRTPlanner::~RRTPlanner() {

}

