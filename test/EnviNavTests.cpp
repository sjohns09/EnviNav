/** @file EnviNavTests.cpp
 * @brief Tests for RRT Planning algorithm
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
#include <geometry_msgs/PoseStamped.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <tf/transform_listener.h>
#include <pluginlib/class_loader.h>
#include <nav_core/base_global_planner.h>
#include <gtest/gtest.h>
#include <ros/ros.h>
#include "RRTPlannerHelper.h"
#include <vector>


class RRTPlannerTester {
 public:
  RRTPlannerTester();
  RRTPlannerHelper getHelper();
  costmap_2d::Costmap2DROS* costmapRos;
};

RRTPlannerTester::RRTPlannerTester() {
  tf::TransformListener tf(ros::Duration(10));
  costmapRos = new costmap_2d::Costmap2DROS("test_costmap", tf);
}

RRTPlannerHelper RRTPlannerTester::getHelper() {
  costmap_2d::Costmap2D* _costmap = costmapRos->getCostmap();

  int _mapSizeX = 10;
  int _mapSizeY = 10;
  float _resolution = 1;
  float _originX = 0;
  float _originY = 0;

  geometry_msgs::PoseStamped givenStartFill;
  geometry_msgs::PoseStamped givenGoalFill;

  givenStartFill.pose.position.x = 1;
  givenStartFill.pose.position.y = 1;
  givenStartFill.pose.position.z = 0;
  givenStartFill.pose.orientation.x = 0;
  givenStartFill.pose.orientation.y = 0;
  givenStartFill.pose.orientation.z = 0;
  givenStartFill.pose.orientation.w = 1;

  givenGoalFill.pose.position.x = 9;
  givenGoalFill.pose.position.y = 9;
  givenGoalFill.pose.position.z = 0;
  givenGoalFill.pose.orientation.x = 0;
  givenGoalFill.pose.orientation.y = 0;
  givenGoalFill.pose.orientation.z = 0;
  givenGoalFill.pose.orientation.w = 1;

  const geometry_msgs::PoseStamped& _goal = givenGoalFill;
  const geometry_msgs::PoseStamped& _start = givenStartFill;

  RRTPlannerHelper helper(_costmap, _mapSizeX, _mapSizeY, _resolution, _originX,
                          _originY, _goal, _start);
  ROS_INFO("Helper Initialized");

  return helper;
}

TEST(RRTPlannerHelper_test, testRandomNodeReturnedInMap) {
  RRTPlannerTester tester;
  RRTPlannerHelper helper = tester.getHelper();

  geometry_msgs::PoseStamped rand = helper.rand_config();

  EXPECT_LE(rand.pose.position.x, helper._mapSizeX);
  EXPECT_LE(rand.pose.position.y, helper._mapSizeY);
}

TEST(RRTPlanner_test, testNearestNodeReturnedFromTreeGraph) {
  RRTPlannerTester tester;
  RRTPlannerHelper helper = tester.getHelper();

  geometry_msgs::PoseStamped q;
  RRTPlannerHelper::qTree node1;
  RRTPlannerHelper::qTree node2;
  RRTPlannerHelper::qTree node3;

  q.pose.position.x = 6;
  q.pose.position.y = 3;

  node1.q.pose.position.x = 1;
  node1.q.pose.position.y = 1;
  node1.myIndex = 0;
  node1.nearIndex = 0;

  node2.q.pose.position.x = 1;
  node2.q.pose.position.y = 8;
  node2.myIndex = 1;
  node2.nearIndex = 2;

  node3.q.pose.position.x = 3;
  node3.q.pose.position.y = 3;
  node3.myIndex = 2;
  node3.nearIndex = 0;

  std::vector<RRTPlannerHelper::qTree> tree;
  tree.push_back(node1);
  tree.push_back(node2);
  tree.push_back(node3);

  int iNear = helper.nearest_vertex(q, tree);

  EXPECT_EQ(iNear, 2);
}

TEST(RRTPlanner_test, testPathBetweenNodesIsSafe) {
  RRTPlannerTester tester;
  RRTPlannerHelper helper = tester.getHelper();

  geometry_msgs::PoseStamped q;
  RRTPlannerHelper::qTree node1;
  RRTPlannerHelper::qTree node2;
  RRTPlannerHelper::qTree node3;

  q.pose.position.x = 6;
  q.pose.position.y = 3;

  node1.q.pose.position.x = 1;
  node1.q.pose.position.y = 1;
  node1.myIndex = 0;
  node1.nearIndex = 0;

  node2.q.pose.position.x = 1;
  node2.q.pose.position.y = 8;
  node2.myIndex = 1;
  node2.nearIndex = 2;

  node3.q.pose.position.x = 3;
  node3.q.pose.position.y = 3;
  node3.myIndex = 2;
  node3.nearIndex = 0;

  std::vector<RRTPlannerHelper::qTree> tree;
  tree.push_back(node1);
  tree.push_back(node2);
  tree.push_back(node3);

  bool safe = helper.path_safe(q, 2, tree);

  EXPECT_EQ(safe, true);
}

TEST(RRTPlanner_test, testPathToGoalIsSafe) {
  RRTPlannerTester tester;
  RRTPlannerHelper helper = tester.getHelper();

  geometry_msgs::PoseStamped q;

  q.pose.position.x = 1;
  q.pose.position.y = 8;

  bool safe = helper.check_goal(q);

  EXPECT_EQ(safe, true);
}

TEST(RRTPlanner_test, testBuildPlanIsCorrect) {
  RRTPlannerTester tester;
  RRTPlannerHelper helper = tester.getHelper();

  geometry_msgs::PoseStamped q;
  RRTPlannerHelper::qTree node1;
  RRTPlannerHelper::qTree node2;
  RRTPlannerHelper::qTree node3;
  RRTPlannerHelper::qTree node4;

  node1.q.pose.position.x = 1;
  node1.q.pose.position.y = 1;
  node1.myIndex = 0;
  node1.nearIndex = 0;

  node2.q.pose.position.x = 1;
  node2.q.pose.position.y = 8;
  node2.myIndex = 1;
  node2.nearIndex = 2;

  node3.q.pose.position.x = 3;
  node3.q.pose.position.y = 3;
  node3.myIndex = 2;
  node3.nearIndex = 0;

  node4.q.pose.position.x = 9;
  node4.q.pose.position.y = 9;
  node4.myIndex = 3;
  node4.nearIndex = 1;

  std::vector<RRTPlannerHelper::qTree> tree;
  tree.push_back(node1);
  tree.push_back(node2);
  tree.push_back(node3);
  tree.push_back(node4);

  std::vector<geometry_msgs::PoseStamped> plan = helper.build_plan(tree);

  EXPECT_EQ(plan[0].pose.position.x, 1);
  EXPECT_EQ(plan[0].pose.position.y, 1);

  EXPECT_EQ(plan[1].pose.position.x, 3);
  EXPECT_EQ(plan[1].pose.position.y, 3);

  EXPECT_EQ(plan[2].pose.position.x, 1);
  EXPECT_EQ(plan[2].pose.position.y, 8);

  EXPECT_EQ(plan[3].pose.position.x, 9);
  EXPECT_EQ(plan[3].pose.position.y, 9);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "EnviNavTests");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
