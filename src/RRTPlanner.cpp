/** @file RRTPlanner.cpp
 * @brief Global path planner implementing the RRT algorithm.
 *
 * @author Samantha Johnson
 * @date December 15, 2017
 * @license BSD 3-Clause License
 * @copyright (c) 2017, Samantha Johnson
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * @details This plugin was created to interface with the nav_core/base_global_planner 
 * framework and replace the default global planner used in the navigation stack. This
 * plugin implements an RRT algorithm which generates a path by creating a tree full of
 * random nodes that are connected to their nearest node neighbor if the path is clear. Once the goal is
 * reached by the tree, the planner returns a path which is a connection of the nodes in
 * the tree that traverse from start to goal.
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
}

RRTPlanner::RRTPlanner(std::string name, costmap_2d::Costmap2DROS* costmapRos) {
  initialize(name, costmapRos);
}

void RRTPlanner::initialize(std::string name,
                            costmap_2d::Costmap2DROS* costmapRos) {
  srand(time(NULL));
  _costmapROS = costmapRos;
  _costmap = costmapRos->getCostmap();

  ros::NodeHandle private_nh("~/" + name);
  // private_nh.param("step_size", _stepSize, _costmap->getResolution());

  _mapSizeX = _costmap->getSizeInCellsX();
  _mapSizeY = _costmap->getSizeInCellsY();
  _resolution = _costmap->getResolution();
  _originX = _costmap->getOriginX();
  _originY = _costmap->getOriginY();

  ROS_INFO(
      "MapSizeX = %f, MapSizeY = %f, Resolution = %f, originX = %f, originY = %f",
      _mapSizeX, _mapSizeY, _resolution, _originX, _originY);

  ROS_INFO("RRT Planner initialized!!");
  _initialized = true;
}

void RRTPlanner::initialize(std::string name, int mapSizeX, int mapSizeY, float resolution,
                            float originX, float originY,
                            costmap_2d::Costmap2DROS map) {
  _mapSizeX = mapSizeX;
  _mapSizeY = mapSizeY;
  _resolution = resolution;
  _originX = originX;
  _originY = originY;
  _costmap = map.getCostmap();

  ros::NodeHandle private_nh("~/" + name);
  // private_nh.param("step_size", _stepSize, _costmap->getResolution());

  _mapSizeX = _costmap->getSizeInCellsX();
  _mapSizeY = _costmap->getSizeInCellsY();
  _resolution = _costmap->getResolution();
  _originX = _costmap->getOriginX();
  _originY = _costmap->getOriginY();

  ROS_INFO(
      "MapSizeX = %f, MapSizeY = %f, Resolution = %f, originX = %f, originY = %f",
      _mapSizeX, _mapSizeY, _resolution, _originX, _originY);

  ROS_INFO("RRT Planner initialized!!");
  _initialized = true;


}

bool RRTPlanner::makePlan(const geometry_msgs::PoseStamped& start,
                          const geometry_msgs::PoseStamped& goal,
                          std::vector<geometry_msgs::PoseStamped>& plan) {
  if (!_initialized) {
    ROS_ERROR(
        "The planner has not been initialized, please call initialize() to use the planner");
    return false;
  }

  ROS_INFO("Got a start: %.2f, %.2f, and a goal: %.2f, %.2f",
           start.pose.position.x, start.pose.position.y, goal.pose.position.x,
           goal.pose.position.y);

  if (goal.header.frame_id != _costmapROS->getGlobalFrameID()) {
    ROS_ERROR(
        "This planner as configured will only accept goals in the %s frame, but a goal was sent in the %s frame.",
        _costmapROS->getGlobalFrameID().c_str(), goal.header.frame_id.c_str());
    return false;
  }

  plan.clear();
  _plan.clear();

  bool done = false;
  bool safe = false;
  bool build = false;
  int iNear;
  geometry_msgs::PoseStamped qRand;

  _start = start;
  _goal = goal;
  _plan = plan;

  // Converting start and goal nodes to Map Coordinates
  double startMapX = _start.pose.position.x;
  double startMapY = _start.pose.position.y;
  double goalMapX = _goal.pose.position.x;
  double goalMapY = _goal.pose.position.y;

  ROS_INFO("Start Rviz (%f, %f) - Goal Rviz (%f, %f)", startMapX, startMapY,
           goalMapX, goalMapY);

  rviz_map(startMapX, startMapY);
  rviz_map(goalMapX, goalMapY);

  ROS_INFO("Start Map (%f, %f) - Goal Map (%f, %f)", startMapX, startMapY,
           goalMapX, goalMapY);

  _start.pose.position.x = startMapX;
  _start.pose.position.y = startMapY;
  _goal.pose.position.x = goalMapX;
  _goal.pose.position.y = goalMapY;

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
  ROS_INFO("Plan Length = %d", plan.size());
  return done;
}

// Get rand x,y in free space of costmap
geometry_msgs::PoseStamped RRTPlanner::rand_config() {

  geometry_msgs::PoseStamped randPose;
  bool found = false;
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

      //_costmap->mapToWorld(randX, randY, worldXRef, worldYRef);

      // Set Arbitrary Orientation
      randPose.pose.position.x = randX;
      randPose.pose.position.y = randY;
      randPose.pose.position.z = 0;

      randPose.pose.orientation.x = 0;
      randPose.pose.orientation.y = 0;
      randPose.pose.orientation.z = 0;
      randPose.pose.orientation.w = 1;

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
  ROS_INFO("Nearest Vertex in tree found (%d, %d)",
           _treeGraph[iNear].q.pose.position.x,
           _treeGraph[iNear].q.pose.position.y);
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
//  unsigned int & xCellRef = xCell;
//  unsigned int & yCellRef = yCell;
  bool free = true;

  double xMax = std::max(x1, x2);
  double xMin = std::min(x1, x2);

  for (double x = xMin; x <= xMax; x++) {
    xCell = (int) round(x);
    y = m * x + b;
    yCell = (int) round(y);

    ROS_INFO("Points On Path Map: X = %d, Y = %d", xCell, yCell);
    ROS_INFO("Points On Path Rviz: X = %f, Y = %f", (xCell * _resolution),
             (yCell * _resolution));

    unsigned char cost = _costmap->getCost(xCell, yCell);
    ROS_INFO("COST = %d", cost);
    if (cost != 0) {
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
//  unsigned int & xCellRef = xCell;
//  unsigned int & yCellRef = yCell;
  bool free = true;

  double xMax = std::max(x1, x2);
  double xMin = std::min(x1, x2);

  for (double x = xMin; x < xMax; x++) {
    y = m * x + b;
    xCell = (int) round(x);
    yCell = (int) round(y);

    ROS_INFO("Points On Goal Path: X = %d, Y = %d", xCell, yCell);
    ROS_INFO("Points On Path Rviz: X = %f, Y = %f", (xCell * _resolution),
             (yCell * _resolution));

    unsigned char cost = _costmap->getCost(xCell, yCell);
    ROS_INFO("COST = %d", cost);
    if (cost != 0) {
      free = false;
      break;
    }
  }
  // Add goal to tree with neighbor qNear if found
  if (free == true) {
    qTree qGoal;
    qGoal.q = _goal;
    qGoal.nearIndex = iNew;
	qGoal.myIndex = _treeGraph.size();
    _treeGraph.push_back(qGoal);
    ROS_INFO("Goal can be reached!");
  }
  return free;
}

// If goal has been reached build plan
bool RRTPlanner::build_plan() {
// Start at goal and add pose and then move to its neighbor pose
  ROS_INFO("Building Plan");
  qTree qAdd = _treeGraph.back();
  double nearI;

  while (qAdd.nearIndex != 0) {
    //Add pose to plan starting with goal
    _plan.insert(_plan.begin(), qAdd.q);

    //Get neighbor of added vertex
    nearI = qAdd.nearIndex;
    qAdd = _treeGraph[nearI];
  }
  // Add start
  _plan.insert(_plan.begin(), _start);

  for (int i = 0; i < _plan.size(); i++) {
    map_rviz(_plan[i].pose.position.x, _plan[i].pose.position.y);
  }
  ROS_INFO("Built Plan");
  return true;
}

void RRTPlanner::rviz_map(double& x, double& y) {
  x = (x - _originX) / _resolution;
  y = (y - _originY) / _resolution;
}
void RRTPlanner::map_rviz(double& x, double& y) {
  x = x * _resolution + _originX;
  y = y * _resolution + _originY;
}

RRTPlanner::~RRTPlanner() {

}

