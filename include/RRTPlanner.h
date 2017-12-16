/** @file RRTPlanner.h
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
#include <pluginlib/class_list_macros.h>
#include <algorithm>
#include <vector>
#include <string>

#ifndef INCLUDE_RRTPLANNER_H_
#define INCLUDE_RRTPLANNER_H_

class RRTPlanner : public nav_core::BaseGlobalPlanner {
 public:
  /**
   * @brief The default constructor for the RRTPlanner
   */
  RRTPlanner();
  /**
   * @brief Constructor for the RRTPlanner
   * @param name The name of the planner node
   * @param costmapRos The costmap to be used in path planning  
   */
  RRTPlanner(std::string name, costmap_2d::Costmap2DROS* costmapRos);

  /**
   * @brief Initializes the planner
   * @param name The name of the planner node
   * @param costmapRos The costmap to be used in path planning  
   */
  void initialize(std::string name, costmap_2d::Costmap2DROS* costmapRos);

  /**
   * @brief Begins the path planner algorithm to generate a plan
   * @param start The start pose of the robot
   * @param goal The goal pose of the robot
   * @return 0 if no plan was found, and 1 if a plan was generated
   */
  bool makePlan(const geometry_msgs::PoseStamped& start,
                const geometry_msgs::PoseStamped& goal,
                std::vector<geometry_msgs::PoseStamped>& plan);
  virtual ~RRTPlanner();

 private:
  std::vector<geometry_msgs::PoseStamped> _plan;
  geometry_msgs::PoseStamped _goal;
  geometry_msgs::PoseStamped _start;
  costmap_2d::Costmap2DROS* _costmapROS;
  costmap_2d::Costmap2D* costmap;
  float resolution, originX, originY;
  int mapSizeX, mapSizeY;
  bool _initialized;
};

#endif /* INCLUDE_RRTPLANNER_H_ */
