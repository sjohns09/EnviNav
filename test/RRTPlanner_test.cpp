/*
 * RRTPlanner_test.cpp
 *
 *  Created on: Dec 9, 2017
 *      Author: sammie
 */

#include "RRTPlanner.h"
#include <gtest/gtest.h>
#include <vector>
#include <geometry_msgs/PoseStamped.h>
#include "ros/ros.h"

// Inject all of the needed variables into the initializer and call the separate functions

TEST(RRTPlanner_test, testPlannerReturnsPathBetweenStartAndGoal) {
  //ros::NodeHandle node;

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
