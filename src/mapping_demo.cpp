/*
 * mapping.cpp
 *
 *  Created on: Dec 4, 2017
 *      Author: sammie
 */

#include <sstream>
#include <string>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "MapBuild.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "mapping_demo");

  MapBuild map;

  std::string a = "";
  std::cout << "In the terminal window, use the keys to drive the robot around and build the map" << std::endl <<
      "When done building map type 'save' and press enter to save map" << std::endl;
  std::cin >> a;
  if (a == "save") {
    map.save_map();
    std::cout << "Press ctrl^C twice in terminal running launch file to Quit";
  } else {
    std::cout << "Press ctrl^C twice in terminal running launch file to Quit";
  }

  ros::spin();

  return 0;
}

