/** @file RRTPlannerHelper.cpp
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
 * @details This class was created to perform the calculations for the RRTPlanner plugin. This
 * class implements an RRT algorithm which generates a path by creating a tree full of
 * random nodes that are connected to their nearest node neighbor if the path is clear. Once the goal is
 * reached by the tree, the planner returns a path which is a connection of the nodes in
 * the tree that traverse from start to goal.
 */
#include "RRTPlannerHelper.h"
#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <algorithm>
#include <vector>

RRTPlannerHelper::RRTPlannerHelper(costmap_2d::Costmap2D* costmap, int mapX,
                                   int mapY, float resolution, float originX,
                                   float originY,
                                   const geometry_msgs::PoseStamped goal,
                                   const geometry_msgs::PoseStamped start) {
  _mapSizeX = mapX;
  _mapSizeY = mapY;
  _resolution = resolution;
  _originX = originX;
  _originY = originY;
  _costmap = costmap;
  _goal = goal;
  _start = start;

  _allowedDist = 100;

  // Converting start and goal nodes to Map Coordinates
  double startMapX = _start.pose.position.x;
  double startMapY = _start.pose.position.y;
  double goalMapX = _goal.pose.position.x;
  double goalMapY = _goal.pose.position.y;

  rviz_map(startMapX, startMapY);
  rviz_map(goalMapX, goalMapY);

  _start.pose.position.x = startMapX;
  _start.pose.position.y = startMapY;
  _goal.pose.position.x = goalMapX;
  _goal.pose.position.y = goalMapY;
}

// Get rand x,y in free space of costmap
geometry_msgs::PoseStamped RRTPlannerHelper::rand_config() {
  geometry_msgs::PoseStamped randPose;
  bool found = false;

  while (!found) {
    // Get Map Size and sample XY in costmap
    int randX = rand() % _mapSizeX;
    int randY = rand() % _mapSizeY;

    if (_costmap->getCost(randX, randY) == 0) {
      found = true;

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
  return randPose;
}

// Get nearest vertex on tree - Euclidean
int RRTPlannerHelper::nearest_vertex(geometry_msgs::PoseStamped qRand,
                                     std::vector<qTree> _treeGraph) {
  int iNear = 0;
  double minDist = 1000;

  for (int node = 0; node < _treeGraph.size(); node++) {
    // Find closest x y
    double xDif = qRand.pose.position.x - _treeGraph[node].q.pose.position.x;
    double yDif = qRand.pose.position.y - _treeGraph[node].q.pose.position.y;

    double dist = sqrt((pow(xDif, 2)) + (pow(yDif, 2)));

    if (dist < minDist) {
      minDist = dist;
      iNear = node;
    }
  }
  if (minDist > _allowedDist) {
    iNear = -1;
  }
  return iNear;
}

// Determine if path is safe
bool RRTPlannerHelper::path_safe(geometry_msgs::PoseStamped qRand, int iNear,
                                 std::vector<qTree> _treeGraph) {
  double x2 = _treeGraph[iNear].q.pose.position.x;
  double y2 = _treeGraph[iNear].q.pose.position.y;
  double x1 = qRand.pose.position.x;
  double y1 = qRand.pose.position.y;
  unsigned int xCell;
  unsigned int yCell;
  bool free = true;

  if (x1 == x2 && y1 == y2) {
    return false;
  }

  if (x1 != x2) {
    double m = (y2 - y1) / (x2 - x1);
    double b = y1 - (m * x1);
    double xMax = std::max(x1, x2);
    double xMin = std::min(x1, x2);

    for (double x = xMin; x <= xMax; x++) {
      xCell = static_cast<int>(round(x));
      double y = m * x + b;
      yCell = static_cast<int>(round(y));

      unsigned char xCost = _costmap->getCost(xCell, yCell);
      if (xCost != 0) {
        free = false;
        return free;
      }
    }
  }

  if (y1 != y2) {
    double yMax = std::max(y1, y2);
    double yMin = std::min(y1, y2);

    if (x1 == x2) {
      for (double y = yMin; y <= yMax; y++) {
        yCell = static_cast<int>(round(y));
        xCell = static_cast<int>(round(x1));

        unsigned char yCost = _costmap->getCost(xCell, yCell);
        if (yCost != 0) {
          free = false;
          return free;
        }
      }
    } else {
      double m = (y2 - y1) / (x2 - x1);
      double b = y1 - (m * x1);
      for (double y = yMin; y <= yMax; y++) {
        yCell = static_cast<int>(round(y));
        double x = (y - b) / m;
        xCell = static_cast<int>(round(x));

        unsigned char yCost = _costmap->getCost(xCell, yCell);
        if (yCost != 0) {
          free = false;
          return free;
        }
      }
    }
  }
  return free;
}

// Check if path is safe - See if goal can be reached
bool RRTPlannerHelper::check_goal(geometry_msgs::PoseStamped qNew) {
  double xDif = qNew.pose.position.x - _goal.pose.position.x;
  double yDif = qNew.pose.position.y - _goal.pose.position.y;

  double dist = sqrt((pow(xDif, 2)) + (pow(yDif, 2)));

  if (dist > _allowedDist) {
    return false;
  }

  double x2 = _goal.pose.position.x;
  double y2 = _goal.pose.position.y;
  double x1 = qNew.pose.position.x;
  double y1 = qNew.pose.position.y;
  double m = (y2 - y1) / (x2 - x1);
  double b = y1 - (m * x1);
  unsigned int xCell;
  unsigned int yCell;
  bool free = true;

  if (x1 == x2 && y1 == y2) {
    return false;
  }

  if (x1 != x2) {
    double xMax = std::max(x1, x2);
    double xMin = std::min(x1, x2);

    for (double x = xMin; x <= xMax; x++) {
      xCell = static_cast<int>(round(x));
      double y = m * x + b;
      yCell = static_cast<int>(round(y));

      unsigned char xCost = _costmap->getCost(xCell, yCell);
      if (xCost != 0) {
        free = false;
        return free;
      }
    }
  }

  if (y1 != y2) {
    if (x1 == x2) {
      double yMax = std::max(y1, y2);
      double yMin = std::min(y1, y2);

      for (double y = yMin; y <= yMax; y++) {
        yCell = static_cast<int>(round(y));
        xCell = static_cast<int>(round(x1));

        unsigned char yCost = _costmap->getCost(xCell, yCell);
        if (yCost != 0) {
          free = false;
          return free;
        }
      }
    } else {
      double m = (y2 - y1) / (x2 - x1);
      double b = y1 - (m * x1);
      double yMax = std::max(y1, y2);
      double yMin = std::min(y1, y2);

      for (double y = yMin; y <= yMax; y++) {
        yCell = static_cast<int>(round(y));
        double x = (y - b) / m;
        xCell = static_cast<int>(round(x));

        unsigned char yCost = _costmap->getCost(xCell, yCell);
        if (yCost != 0) {
          free = false;
          return free;
        }
      }
    }
  }
  return free;
}

// If goal has been reached build plan
std::vector<geometry_msgs::PoseStamped> RRTPlannerHelper::build_plan(
    std::vector<qTree> _treeGraph) {
// Start at goal and add pose and then move to its neighbor pose
  ROS_INFO("Building Plan");
  _plan.clear();
  qTree qAdd = _treeGraph.back();

  while (qAdd.myIndex != 0) {
    // Add pose to plan starting with goal
    _plan.insert(_plan.begin(), qAdd.q);

    // Get neighbor of added vertex
    double nearI = qAdd.nearIndex;
    qAdd = _treeGraph[nearI];
  }
  // Add start to plan
  _plan.insert(_plan.begin(), _start);

  for (int i = 0; i < _plan.size(); i++) {
    map_rviz(_plan[i].pose.position.x, _plan[i].pose.position.y);
  }
  return _plan;
}

void RRTPlannerHelper::rviz_map(double& x, double& y) {
  x = (x - _originX) / _resolution;
  y = (y - _originY) / _resolution;
}
void RRTPlannerHelper::map_rviz(double& x, double& y) {
  x = x * _resolution + _originX;
  y = y * _resolution + _originY;
}

