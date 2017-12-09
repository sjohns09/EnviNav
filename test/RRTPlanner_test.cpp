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

TEST(RRTPlanner_test, testPlannerReturnsPathBetweenStartAndGoal) {
  ros::NodeHandle node;

  geometry_msgs::PoseStamped givenStart;
  geometry_msgs::PoseStamped givenGoal;

  //This variable holds the final plan in the global planner when called
  std::vector<geometry_msgs::PoseStamped>& plan;
  // Need to call planner here
  // ------

  auto start = plan.begin();
  auto goal = plan.end();

  EXPECT_EQ(start, givenStart);
  EXPECT_EQ(goal, givenGoal);
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
