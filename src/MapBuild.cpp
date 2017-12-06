/*
 * MapBuild.cpp
 *
 *  Created on: Dec 5, 2017
 *      Author: sammie
 */
#include <sstream>
#include <string>
#include "std_msgs/String.h"
#include "MapBuild.h"

MapBuild::MapBuild() {
  // TODO Auto-generated constructor stub

}

int MapBuild::save_map() {

  system("rosrun map_server map_saver -f $(rospack find envi_nav)/maps/home");
  std::cout << "MAP SAVED" << std::endl;

  return 1;

}

MapBuild::~MapBuild() {
  // TODO Auto-generated destructor stub
}

