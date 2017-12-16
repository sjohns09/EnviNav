/** @file mapping_demo.cpp
 * @brief Gives instructions for building the navigation map.
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
 * @details Provides detailed instructions for building the map that is used with the
 * navigation demo.
 */
#include <ros/ros.h>
#include "MapBuild.h"
#include <sstream>
#include <string>

int main(int argc, char **argv) {
  ros::init(argc, argv, "mapping_demo");
  ros::NodeHandle n;
  MapBuild map;

  std::string a = "";
  ROS_INFO(
      "In the terminal window, use the keyboard to drive the robot around to build the map");
  ROS_INFO(
      "Useful Keys: 'I' or ',' - Forward/Backward, 'J' or 'L' - Rotate, 'Q' or 'Z' - Inc/Dec Speed");
  ROS_INFO(
      "When done building the map, type 'save' in this window and press enter");

  std::cin >> a;
  if (a == "save") {
    map.save_map();
  } else {
    std::cout << "Press ctrl^C twice in terminal running launch file to quit";
  }

  ros::spin();

  return 0;
}

