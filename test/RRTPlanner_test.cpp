/** @file RRTPlanner_test.cpp
 * @brief Test for RRTPlanner plugin
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
 * @details Tests for the RRT algorithm used as a global path planner plugin
 * to the navigation stack
 */

#include "RRTPlanner.h"
#include <gtest/gtest.h>
#include <vector>
#include <geometry_msgs/PoseStamped.h>
#include "ros/ros.h"

// Inject all of the needed variables into the initializer and call the separate functions

/**
* @test Verifies that the start and goal points and the first and last poses in the plan
*/
TEST(RRTPlanner_test, testPlannerReturnsPathBetweenStartAndGoal) {
  //ros::NodeHandle node;
  // Launch the map server node

  //This variable holds the final plan in the global planner when called
  std::vector<geometry_msgs::PoseStamped>& plan = {};
  geometry_msgs::PoseStamped givenStart;
  geometry_msgs::PoseStamped givenGoal;

  givenStart.pose.position.x = 1;
  givenStart.pose.position.y = 1;
  givenStart.pose.position.z = 0;
  givenStart.pose.orientation.x = 0;
  givenStart.pose.orientation.y = 0;
  givenStart.pose.orientation.z = 0;
  givenStart.pose.orientation.w = 1;

  givenGoal.pose.position.x = 3;
  givenGoal.pose.position.y = 3;
  givenGoal.pose.position.z = 0;
  givenGoal.pose.orientation.x = 0;
  givenGoal.pose.orientation.y = 0;
  givenGoal.pose.orientation.z = 0;
  givenGoal.pose.orientation.w = 1;

  // Need to call planner here
  tf::TransformListener tf;
  costmap_2d::Costmap2DROS* costmap = new costmap_2d::Costmap2DROS(
      "test_costmap", tf);
  RRTPlanner planner = new RRTPlanner("rrt_planner", costmap);
  planner.initialize("rrt_planner", costmap);
  planner.makePlan(givenStart, givenGoal, plan);
  // ------

  geometry_msgs::PoseStamped startPlan = plan.front();
  geometry_msgs::PoseStamped goalPlan = plan.back();

  EXPECT_EQ(startPlan.pose.position.x, givenStart.pose.position.x);
  EXPECT_EQ(startPlan.pose.position.y, givenStart.pose.position.y);
  EXPECT_EQ(goalPlan.pose.position.x, givenGoal.pose.position.x);
  EXPECT_EQ(goalPlan.pose.position.y, givenGoal.pose.position.y);
}

/**
* @test Verifies that all nodes in plan are in free space
*/
TEST(RRTPlanner_test, testPlannerDoesNotCauseCollisions) {
  ros::NodeHandle node;

  // Check to make sure every cell in plan is free?

  EXPECT_EQ("", "");
}

TEST(RRTPlanner_test, test) {
  ros::NodeHandle node;

  EXPECT_EQ("", "");
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "RRTPlanner_test");

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
