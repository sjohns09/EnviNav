/*
 * RRTPlannerHelper.h
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
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>

#ifndef ENVI_NAV_INCLUDE_RRTPLANNERHELPER_H_
#define ENVI_NAV_INCLUDE_RRTPLANNERHELPER_H_

class RRTPlannerHelper {
 public:
  RRTPlannerHelper(costmap_2d::Costmap2D* costmap, int& mapX, int& mapY,
                   float& resolution, float& originX, float& originY,
                   const geometry_msgs::PoseStamped& goal);
  virtual ~RRTPlannerHelper();

  /**
   * @struct qTree
   * @brief Structure that holds the data related to each node in the tree
   */
  struct qTree {
    geometry_msgs::PoseStamped q;
    geometry_msgs::PoseStamped qNear;
    int myIndex;
    int nearIndex;
  };

  /**
   * @brief Gets a random X and Y coordinate to create a new noded in the tree
   * @return A random configuration in the costmap
   */
  geometry_msgs::PoseStamped rand_config();

  /**
   * @brief Gets the nearest node in the tree
   * @param qRand The randomly generated pose for the robot
   * @return The index of the nearest vertex in the tree
   */
  int nearest_vertex(geometry_msgs::PoseStamped qRand,
                     std::vector<qTree> treeGraph);

  /**
   * @brief Checks that the path between the two nodes is safe
   * @param qRand The new random configuration
   * @param iNear The index of the nearest node in the tree
   * @return 0 if the path is blocked, and 1 if the path is clear
   */
  bool path_safe(geometry_msgs::PoseStamped qRand, int iNear,
                 std::vector<qTree> _treeGraph);

  /**
   * @brief Checks that the path between the newly added node and the goal is safe
   * @param qNew The new configuration added to the tree
   * @param iNew The index of the node being checked for goal
   * @return 0 if path is blocked, and 1 if the path is clear
   */
  bool check_goal(geometry_msgs::PoseStamped qNew);

  /**
   * @brief Builds the planned path from the node tree
   * @return 0 if plan could not be built, and 1 if a plan was generated
   */
  std::vector<geometry_msgs::PoseStamped> build_plan(
      std::vector<qTree> _treeGraph);

  /**
   * @brief Converts the coordinates given from rviz to costmap coordinates
   * @param x The x component of the coordinate
   * @param y The y component of the coordinate
   */
  void rviz_map(double& x, double& y);

  /**
   * @brief Converts the coordinates from the costmap to rviz coordinates
   * @param x The x component of the coordinate
   * @param y The y component of the coordinate
   */
  void map_rviz(double& x, double& y);

  std::vector<geometry_msgs::PoseStamped> _plan;
  geometry_msgs::PoseStamped _goal;
  geometry_msgs::PoseStamped _start;
  costmap_2d::Costmap2D* _costmap;
  float _resolution, _originX, _originY;
  int _mapSizeX, _mapSizeY;
  double _allowedDist;
};

#endif /* ENVI_NAV_INCLUDE_RRTPLANNERHELPER_H_ */
