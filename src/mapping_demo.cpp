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
  int done = 0;
  std::cout << "When done enter 'save' or to exit without saving enter 'exit'"
            << std::endl;

  while (done != 1) {
    std::cin >> a;
    if (a == "save") {
      done = map.save_map();

    } else if (a == "exit") {
      done = 1;
    }
  }
  std::cout << "Press ctrl^C twice in terminal running launch file to Quit";

  ros::spin();

  return 0;
}

