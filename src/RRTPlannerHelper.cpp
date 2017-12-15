/*
 * RRTPlannerHelper.cpp
 *
 *  Created on: Dec 15, 2017
 *      Author: sammie
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
#include <pluginlib/class_list_macros.h>
#include "RRTPlannerHelper.h"

RRTPlannerHelper::RRTPlannerHelper(costmap_2d::Costmap2D* costmap, int mapX,
                                   int mapY, float resolution, float originX,
                                   float originY,
                                   const geometry_msgs::PoseStamped goal, const geometry_msgs::PoseStamped start) {
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
}

// Get rand x,y in free space of costmap
geometry_msgs::PoseStamped RRTPlannerHelper::rand_config() {

  geometry_msgs::PoseStamped randPose;
  bool found = false;
  int randX;
  int randY;

  while (!found) {
    // Get Map Size and sample XY in costmap

    randX = rand() % _mapSizeX;
    randY = rand() % _mapSizeY;

    if (_costmap->getCost(randX, randY) == 0) {
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
int RRTPlannerHelper::nearest_vertex(geometry_msgs::PoseStamped qRand,
                                     std::vector<qTree> _treeGraph) {
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

    dist = sqrt((pow(xDif, 2)) + (pow(yDif, 2)));

    if (dist < minDist) {
      minDist = dist;
      iNear = node;  //_treeGraph[node].myIndex;
    }
  }
  ROS_INFO("Nearest Vertex in tree found (%f, %f)",
           _treeGraph[iNear].q.pose.position.x,
           _treeGraph[iNear].q.pose.position.y);
  ROS_INFO("Distance = %f", minDist);

  if (minDist > _allowedDist) {
    iNear = -1;
  }

  return iNear;
}

// Determine if path is safe
bool RRTPlannerHelper::path_safe(geometry_msgs::PoseStamped qRand, int iNear,
                                 std::vector<qTree> _treeGraph) {
  ROS_INFO("Checking Path");
  double x2 = _treeGraph[iNear].q.pose.position.x;
  double y2 = _treeGraph[iNear].q.pose.position.y;
  double x1 = qRand.pose.position.x;
  double y1 = qRand.pose.position.y;
  double y, x;
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
      xCell = (int) round(x);
      y = m * x + b;
      yCell = (int) round(y);

      ROS_INFO("Points On Path Map: X = %d, Y = %d", xCell, yCell);
      ROS_INFO("Points On Path Rviz: X = %f, Y = %f", (xCell * _resolution),
               (yCell * _resolution));

      unsigned char xCost = _costmap->getCost(xCell, yCell);
      ROS_INFO("COST = %d", xCost);
      if (xCost != 0) {
        free = false;
        ROS_INFO("Path is not safe");
        return free;
      }
    }
  }

  if (y1 != y2) {
    double yMax = std::max(y1, y2);
    double yMin = std::min(y1, y2);

    if (x1 == x2) {
      for (double y = yMin; y <= yMax; y++) {
        yCell = (int) round(y);
        xCell = (int) round(x1);

        ROS_INFO("Points On Path Map: X = %d, Y = %d", xCell, yCell);
        ROS_INFO("Points On Path Rviz: X = %f, Y = %f", (xCell * _resolution),
                 (yCell * _resolution));

        unsigned char yCost = _costmap->getCost(xCell, yCell);
        ROS_INFO("COST = %d", yCost);
        if (yCost != 0) {
          free = false;
          ROS_INFO("Path is not safe");
          return free;
        }
      }
    } else {
      double m = (y2 - y1) / (x2 - x1);
      double b = y1 - (m * x1);
      for (double y = yMin; y <= yMax; y++) {
        yCell = (int) round(y);
        x = (y - b) / m;
        xCell = (int) round(x);

        ROS_INFO("Points On Path Map: X = %d, Y = %d", xCell, yCell);
        ROS_INFO("Points On Path Rviz: X = %f, Y = %f", (xCell * _resolution),
                 (yCell * _resolution));

        unsigned char yCost = _costmap->getCost(xCell, yCell);
        ROS_INFO("COST = %d", yCost);
        if (yCost != 0) {
          free = false;
          ROS_INFO("Path is not safe");
          return free;
        }
      }
    }
  }
  return free;
}

// Check if path is safe - See if goal can be reached
bool RRTPlannerHelper::check_goal(geometry_msgs::PoseStamped qNew) {
  ROS_INFO("Checking Goal Distance");

  double xDif = qNew.pose.position.x - _goal.pose.position.x;
  double yDif = qNew.pose.position.y - _goal.pose.position.y;

  double dist = sqrt((pow(xDif, 2)) + (pow(yDif, 2)));

  if (dist > _allowedDist) {
    ROS_INFO("Goal too far: %f", dist);
    return false;
  }

  ROS_INFO("Checking Goal Path");

  double x2 = _goal.pose.position.x;
  double y2 = _goal.pose.position.y;
  double x1 = qNew.pose.position.x;
  double y1 = qNew.pose.position.y;
  double m = (y2 - y1) / (x2 - x1);
  double b = y1 - (m * x1);
  double y, x;
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
      xCell = (int) round(x);
      y = m * x + b;
      yCell = (int) round(y);

      ROS_INFO("Points On Path Map: X = %d, Y = %d", xCell, yCell);
      ROS_INFO("Points On Path Rviz: X = %f, Y = %f", (xCell * _resolution),
               (yCell * _resolution));

      unsigned char xCost = _costmap->getCost(xCell, yCell);
      ROS_INFO("COST = %d", xCost);
      if (xCost != 0) {
        free = false;
        ROS_INFO("Path is not safe");
        return free;
      }
    }
  }

  if (y1 != y2) {
      if (x1 == x2) {
        double yMax = std::max(y1, y2);
        double yMin = std::min(y1, y2);

        for (double y = yMin; y <= yMax; y++) {
          yCell = (int) round(y);
          xCell = (int) round(x1);

          ROS_INFO("Points On Path Map: X = %d, Y = %d", xCell, yCell);
          ROS_INFO("Points On Path Rviz: X = %f, Y = %f", (xCell * _resolution),
                   (yCell * _resolution));

          unsigned char yCost = _costmap->getCost(xCell, yCell);
          ROS_INFO("COST = %d", yCost);
          if (yCost != 0) {
            free = false;
            ROS_INFO("Path is not safe");
            return free;
          }
        }
      } else {
        double m = (y2 - y1) / (x2 - x1);
        double b = y1 - (m * x1);
        double yMax = std::max(y1, y2);
        double yMin = std::min(y1, y2);

        for (double y = yMin; y <= yMax; y++) {
          yCell = (int) round(y);
          x = (y - b) / m;
          xCell = (int) round(x);

          ROS_INFO("Points On Path Map: X = %d, Y = %d", xCell, yCell);
          ROS_INFO("Points On Path Rviz: X = %f, Y = %f", (xCell * _resolution),
                   (yCell * _resolution));

          unsigned char yCost = _costmap->getCost(xCell, yCell);
          ROS_INFO("COST = %d", yCost);
          if (yCost != 0) {
            free = false;
            ROS_INFO("Path is not safe");
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
  double nearI;

  while (qAdd.myIndex != 0) {
    //Add pose to plan starting with goal
    _plan.insert(_plan.begin(), qAdd.q);

    //Get neighbor of added vertex
    nearI = qAdd.nearIndex;
    qAdd = _treeGraph[nearI];
  }
  // Add start to plan
  _plan.insert(_plan.begin(), _start);

  for (int i = 0; i < _plan.size(); i++) {
    map_rviz(_plan[i].pose.position.x, _plan[i].pose.position.y);
  }
  ROS_INFO("Built Plan");
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

