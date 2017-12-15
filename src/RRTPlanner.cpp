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
 * plugin uses an RRT algorithm which generates a path by creating a tree full of
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
#include <vector>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>
#include <pluginlib/class_list_macros.h>
#include "RRTPlanner.h"
#include <RRTPlannerHelper.h>

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
  costmap = costmapRos->getCostmap();

  ros::NodeHandle private_nh("~/" + name);

  mapSizeX = costmap->getSizeInCellsX();
  mapSizeY = costmap->getSizeInCellsY();
  resolution = costmap->getResolution();
  originX = costmap->getOriginX();
  originY = costmap->getOriginY();

  ROS_INFO("RRT Planner initialized!");
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

  ROS_INFO("Got a start: (%f, %f), and a goal: (%f, %f)",
           start.pose.position.x, start.pose.position.y, goal.pose.position.x,
           goal.pose.position.y);

  std::vector<RRTPlannerHelper::qTree> treeGraph;
  RRTPlannerHelper helper(costmap, mapSizeX, mapSizeY, resolution,
                                originX, originY, goal, start);
  plan.clear();

  bool done = false;
  bool safe = false;
  bool build = false;
  int iNear;
  geometry_msgs::PoseStamped qRand;

  _start = start;
  _goal = goal;

  // Converting start and goal nodes to Map Coordinates
  double startMapX = _start.pose.position.x;
  double startMapY = _start.pose.position.y;
  double goalMapX = _goal.pose.position.x;
  double goalMapY = _goal.pose.position.y;

  helper.rviz_map(startMapX, startMapY);
  helper.rviz_map(goalMapX, goalMapY);

  _start.pose.position.x = startMapX;
  _start.pose.position.y = startMapY;
  _goal.pose.position.x = goalMapX;
  _goal.pose.position.y = goalMapY;

  ROS_INFO("Making Navigation Plan Using RRT!");

  // Add start node to tree
  RRTPlannerHelper::qTree startQ;
  startQ.q = _start;
  startQ.qNear = _start;
  startQ.nearIndex = 0;
  startQ.myIndex = 0;
  treeGraph.push_back(startQ);

  // RRT Algorithm
  while (done == false) {
    // Get Random Pose
    qRand = helper.rand_config();

    // Find nearest Vertex in tree
    iNear = helper.nearest_vertex(qRand, treeGraph);

    // If distance is too far, do not add node
    if (iNear != -1) {
      // Check if path is safe
      safe = helper.path_safe(qRand, iNear, treeGraph);

      // If path is safe add node to tree
      if (safe == true) {
        RRTPlannerHelper::qTree addQ;
        addQ.q = qRand;
        addQ.qNear = treeGraph[iNear].q;
        addQ.nearIndex = iNear;
        addQ.myIndex = treeGraph.size();
        treeGraph.push_back(addQ);

        // Check if goal can be reached
        build = helper.check_goal(addQ.q);

        // If goal can be reached and path is safe
        if (build == true) {
          // Add goal to tree with neighbor qNear if found
          RRTPlannerHelper::qTree qGoal;
          qGoal.q = _goal;
          qGoal.qNear = qRand;
          qGoal.nearIndex = addQ.myIndex;
          qGoal.myIndex = treeGraph.size();
          treeGraph.push_back(qGoal);
          ROS_INFO("Goal can be reached!");

          plan = helper.build_plan(treeGraph);
          done = true;
        }
      }
    }
  }
  ROS_INFO("New Plan Created");
  ROS_INFO("Plan Length = %d nodes", plan.size());
  return done;
}

RRTPlanner::~RRTPlanner() {

}

