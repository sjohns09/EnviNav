/*
 * RRTPlanner.cpp
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

  ros::NodeHandle private_nh("~/" + name);
  private_nh.param("step_size", _stepSize, _costmap->getResolution());

  ROS_INFO("RRT Planner initialized!!");

}

bool RRTPlanner::makePlan(const geometry_msgs::PoseStamped& start,
                          const geometry_msgs::PoseStamped& goal,
                          std::vector<geometry_msgs::PoseStamped>& plan) {
  bool done = false;
  bool safe = false;
  bool build = false;
  int iNear;
  geometry_msgs::PoseStamped qRand;

  _sampleRange = 2;
  _stepSize = 0.2;
  _start = start;
  _goal = goal;
  _plan = plan;

  ROS_INFO("Making Navigation Plan!");

  // Add start node to tree
  qTree startQ;
  startQ.q = _start;
  startQ.qNear = _start;
  startQ.nearIndex = 0;

  _treeGraph.push_back(startQ);

  // RRT Algorithm
  while (done == false) {
    // Get Random Pose
    qRand = rand_config();

    // Find nearest Vertex in tree
    iNear = nearest_vertex(qRand);

    // Check if path is safe
    safe = path_safe(qRand, iNear);

    // If path is safe add node to tree
    if (safe == true) {
      qTree addQ;
      addQ.q = qRand;
      addQ.qNear = _treeGraph[iNear].q;
      addQ.nearIndex = iNear;
      _treeGraph.push_back(addQ);

      // Check if goal can be reached
      build = check_goal(addQ.q);

      // If goal can be reached and path is safe
      if (build == true) {
        done = build_plan();  //Will return true when done
      }
    }
  }
  plan = _plan;
  return done;
}

// Get rand x,y in free space of costmap
geometry_msgs::PoseStamped RRTPlanner::rand_config() {
  geometry_msgs::PoseStamped randPose;

  // Get Map Size and sample XY in free space (in range?)

  // Set Arbitrary Orientation

  return randPose;
}

// Get nearest vertex on tree - Euclidean
int RRTPlanner::nearest_vertex(geometry_msgs::PoseStamped qRand) {
  int iNear;
  for (qTree node : _treeGraph) {
    // Find closest x y
  }

  return iNear;
}

// Determine if path is safe using step size to check path
bool RRTPlanner::path_safe(geometry_msgs::PoseStamped qRand, int iNear) {
  _stepSize;
  return false;
}

// Check if path is safe - See if goal can be reached within max sampling distance
bool RRTPlanner::check_goal(geometry_msgs::PoseStamped qNew) {

  // Add goal to tree with neighbor qNear if found
  return false;
}

// If goal has been reached build plan
bool RRTPlanner::build_plan() {
// Start at goal and add pose and then move to its neighbor pose
  qTree qAdd = _treeGraph.back();
  double nearI;

  while (qAdd.nearIndex != 0) {
    //Add pose to plan starting with goal
    _plan.insert(_plan.begin(), qAdd.q);

    //Get neighbor of added vertex
    nearI = qAdd.nearIndex;
    qAdd = _treeGraph[nearI];
  }
  return true;
}

RRTPlanner::~RRTPlanner() {

}

