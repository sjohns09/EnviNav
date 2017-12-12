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
#include <algorithm>
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

//  ros::NodeHandle private_nh("~/" + name);
//  private_nh.param("step_size", _stepSize, _costmap->getResolution());

  _mapSizeX = _costmap->getSizeInCellsX();
  _mapSizeY = _costmap->getSizeInCellsY();
  _stepSize = 1; //_costmap->getResolution();

  ROS_INFO("RRT Planner initialized!!");

}

bool RRTPlanner::makePlan(const geometry_msgs::PoseStamped& start,
                          const geometry_msgs::PoseStamped& goal,
                          std::vector<geometry_msgs::PoseStamped>& plan) {
  plan.clear();
  _plan.clear();

  bool done = false;
  bool safe = false;
  bool build = false;
  int iNear;
  geometry_msgs::PoseStamped qRand;

  _sampleRange = 2;
  _start = start;
  _goal = goal;
  _plan = plan;

  ROS_INFO("Making Navigation Plan!");

  // Add start node to tree
  qTree startQ;
  startQ.q = _start;
  startQ.qNear = _start;
  startQ.nearIndex = 0;
  startQ.myIndex = 0;

  _treeGraph.push_back(startQ);

  // RRT Algorithm
  while (done == false) {
    ROS_INFO("New Iteration");
    // Get Random Pose
    qRand = rand_config();
    ROS_INFO("Back Here");

    // Find nearest Vertex in tree
    iNear = nearest_vertex(qRand);
    ROS_INFO("Back Here");

    // Check if path is safe
    safe = path_safe(qRand, iNear);
    ROS_INFO("Back Here");

    // If path is safe add node to tree
    if (safe == true) {
      qTree addQ;
      addQ.q = qRand;
      addQ.qNear = _treeGraph[iNear].q;
      addQ.nearIndex = iNear;
      addQ.myIndex = _treeGraph.size();
      _treeGraph.push_back(addQ);

      // Check if goal can be reached
      build = check_goal(addQ.q, addQ.myIndex);

      // If goal can be reached and path is safe
      if (build == true) {
        done = build_plan();  //Will return true when done
      }
    }
  }
  plan = _plan;
  ROS_INFO("Plan Created");
  return done;
}

// Get rand x,y in free space of costmap
geometry_msgs::PoseStamped RRTPlanner::rand_config() {

  geometry_msgs::PoseStamped randPose;
  bool found = false;
  srand(time(NULL));
  int randX;
  int randY;
  double worldX = 0;
  double worldY = 0;
  double & worldXRef = worldX;
  double & worldYRef = worldY;

  while (!found) {
    // Get Map Size and sample XY in costmap

    randX = rand() % _mapSizeX;
    randY = rand() % _mapSizeY;

    if (_costmap->getCost(randX, randY) == costmap_2d::FREE_SPACE) {
      found = true;

      _costmap->mapToWorld(randX, randY, worldXRef, worldYRef);

      // Set Arbitrary Orientation
      randPose.pose.position.x = worldX;
      randPose.pose.position.y = worldY;
      randPose.pose.position.z = 0;

      randPose.pose.orientation.x = 0;
      randPose.pose.orientation.y = 0;
      randPose.pose.orientation.z = 0;
      randPose.pose.orientation.w = 0;

    } else {
      found = false;
    }
  }
  ROS_INFO("New Random Pose Found!: X = %d, Y = %d", randX, randY);
  return randPose;
}

// Get nearest vertex on tree - Euclidean
int RRTPlanner::nearest_vertex(geometry_msgs::PoseStamped qRand) {
  ROS_INFO("Looking for Nearest Vertex in tree");
  int iNear = 0;
  double minDist = 1000;
  double dist;
  double xDif;
  double yDif;

  for (int node = 0; node < _treeGraph.size(); node++) {
    // Find closest x y
    xDif = qRand.pose.position.x - _treeGraph[node].q.pose.position.x;
    yDif = qRand.pose.position.y - _treeGraph[node].q.pose.position.y;

    dist = sqrt((xDif * xDif) + (yDif * yDif));

    if (dist < minDist) {
      minDist = dist;
      iNear = _treeGraph[node].myIndex;
    }
  }
  ROS_INFO("Nearest Vertex in tree found");
  return iNear;
}

// Determine if path is safe using step size to check path
bool RRTPlanner::path_safe(geometry_msgs::PoseStamped qRand, int iNear) {
  ROS_INFO("Checking Path");
  double x2 = _treeGraph[iNear].q.pose.position.x;
  double y2 = _treeGraph[iNear].q.pose.position.y;
  double x1 = qRand.pose.position.x;
  double y1 = qRand.pose.position.y;
  double m = (y2 - y1) / (x2 - x1);
  double b = y1 - (m * x1);
  double y;
  unsigned int xCell = 0;
  unsigned int yCell = 0;
  unsigned int & xCellRef = xCell;
  unsigned int & yCellRef = yCell;
  bool free = true;

  double xMax = std::max(x1, x2);
  double xMin = std::min(x1, x2);

  for (double x = xMin; x < xMax; x++) {
    y = m * x + b;
    ROS_INFO("Points On Path: X = %d, Y = %d", x, y);
    _costmap->worldToMap(x, y, xCellRef, yCellRef);
    ROS_INFO("Cells On Path: X = %d, Y = %d", xCell, yCell);
    if (_costmap->getCost(xCell, yCell) == costmap_2d::LETHAL_OBSTACLE) {
      free = false;
      ROS_INFO("Path is not safe");
      break;
    }
  }
  return free;
}

// Check if path is safe - See if goal can be reached
bool RRTPlanner::check_goal(geometry_msgs::PoseStamped qNew, int iNew) {
  ROS_INFO("Checking Goal");
  double x2 = _goal.pose.position.x;
  double y2 = _goal.pose.position.y;
  double x1 = qNew.pose.position.x;
  double y1 = qNew.pose.position.y;
  double m = (y2 - y1) / (x2 - x1);
  double b = y1 - (m * x1);
  double y;
  unsigned int xCell = 0;
  unsigned int yCell = 0;
  unsigned int & xCellRef = xCell;
  unsigned int & yCellRef = yCell;
  bool free = true;

  double xMax = std::max(x1, x2);
  double xMin = std::min(x1, x2);

  for (double x = xMin; x < xMax; x++) {
    y = m * x + b;
    _costmap->worldToMap(x, y, xCellRef, yCellRef);
    if (_costmap->getCost(xCell, yCell) == costmap_2d::LETHAL_OBSTACLE) {
      free = false;
      break;
    }
  }
  // Add goal to tree with neighbor qNear if found
  if (free == true) {
    qTree qGoal;
    qGoal.q = _goal;
    qGoal.nearIndex = iNew;
    _treeGraph.push_back(qGoal);
    ROS_INFO("Goal can be reached!");
  }
  return free;
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
  ROS_INFO("Built Plan");
  return true;
}

RRTPlanner::~RRTPlanner() {

}

