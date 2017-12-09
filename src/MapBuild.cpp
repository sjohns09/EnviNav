/*
 * MapBuild.cpp
 *
 *  Created on: Dec 5, 2017
 *      Author: sammie
 */
#include <sstream>
#include <string>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "MapBuild.h"

MapBuild::MapBuild() {
  // TODO Auto-generated constructor stub

}

void MapBuild::save_map() {
  system("rosrun map_server map_saver -f $(rospack find envi_nav)/maps/home");
  ROS_INFO("Map Saved!");
  std::cout << "Press ctrl^C twice in terminal running launch file to Quit";
}

MapBuild::~MapBuild() {
  // TODO Auto-generated destructor stub
}

