# EnviNav
The Navigation Module for Mental Health RoboPet Companion!

[![Build Status](https://travis-ci.org/sjohns09/envi_nav.svg?branch=Sprint3)](https://travis-ci.org/sjohns09/envi_nav)

ACME Robotics is developing an in home mental health robotic pet companion. This companion is geared toward individuals with moderate-severe mental health conditions that would benefit from constant supervision and stimulation from an external source. The target consumer is a young to middle aged adult, who may live alone or are in a situation where there is limited supervision, and who also suffers from a behavioral disorder such as depression, anxiety, or bipolar disorder. The goal of this product is to provide companionship, motivation, emotional stabilization, activity notifications, and emergency resources to aid in the recovery process for an individual suffering from a mental health illness. This product hopes to speed up the recovery time and decrease the relapse rate for these individuals, and also reduce the high number of suicides that occur amongst this group.

The developed product will be a wheeled mobile pet-like robot that will be capable of interacting with its human companion through voice commands, wearable technology, and facial expressions and gestures. Previous development has occurred on the Mental State Reactionary Decision Maker (MSRDM) component of this robot and a demo was provided that displayed the robot’s ability to react to various states of human emotion as well as learn what type of interaction its human prefers. For this development cycle the focus will be on the navigation aspect of the RoboPet, which will be called EnviNav. This is a crucial component of the system because the robot needs to learn about its environment, which would be the home of the human, so that it can effectively assist the human as necessary. The robot will need to not only gather and store information about its environment, but also autonomously navigate to various locations, whether directed by voice command or by beacon, without colliding into known or unknown obstacles. The learning capabilities of the RoboPet will allow it to be tailored specifically to its human and its environment so that it can be integrated seamlessly into its human’s life.

EnviNav will initialize the robot by allowing it to discover its environment and record a base map. To do this the robot will use the SLAM method. The robot will be guided around the environment and allowed to record and save the map it creates using a laser sensor. Once the robot has discovered its environment it will be able to localize itself to allow it to travel to given locations on the map. The robot will be equipped with a global planner, which will utilize the RRT algorithm and allow the robot to plan a path to its destination. It will also be equipped with a local planner, which will allow the robot to avoid unknown obstacles that may appear along its path, and then successfully migrate back to the global path plan. This module is being developed, ran, and tested using a simulated Turtlebot in a custom Gazebo world environment. 

## Project Backlog and Sprint Backlog
All issues for the project backlog are being tracked in the issues section of this repo (https://github.com/sjohns09/EnviNav/issues).

The sprint backlog is located in the projects section where each sprint has its own project and issues tied to milestones which help track the due date for each sprint (https://github.com/sjohns09/EnviNav/projects).

## Worklog and Sprint Review Notes

The Log for this project is located at:
https://docs.google.com/a/terpmail.umd.edu/spreadsheets/d/1aHDHUWVx5oCtyFJUyxdDZbs8FFohRLv_YU-jJhAd0oE/edit?usp=sharing

The first tab of this sheet contains the Worklog, and the second tab contains the Review notes from the post sprint reviews.

## Dependencies

 - ROS Kinetic (with Gazebo) is installed on machine and running natively
 - The Turtlebot navigation and gazebo packages are installed
 - Rviz is installed on machine
 - The workstation is running with a dedicated GPU which can handle the simulation rendering
 - The "envi_nav" package is cloned in a configured catkin workspace

## Build

 - Clone repo (https://github.com/sjohns09/envi_nav.git) to catkin workspace
 - Ensure that master branch is checked out
 - In catkin workspace root directory, run catkin_make. This will build the "envi_nav" project.
 
 ```
 cd ~/catkin_ws/src
 git clone https://github.com/sjohns09/envi_nav.git
 cd envi_nav
 git checkout master
 cd ~/catkin_ws
 catkin_make
 ```
 
## Demos

The envi_nav module consists of two parts which each contain demos that are dependent on one another. Both of these parts work together to create an optimal navigation experience for the robot. Details on each demo's functionality is discussed below.

### Mapping Demo

The mapping demo is the first step in the envi_nav module. The robot needs to discover its environment before it can accurately navigate to various locations. To discover its environment, the robot will be driven around the world while using laser scanning to determine safe and unsafe areas of its world.

The mapping demo will launch the turtlebot in the custom gazebo world which simulates an apartment environment. This will also launch rviz, to use for visualization of the built map, gmapping, which will allow the robot to conduct SLAM on its environment, and the keyboard teleop node which allows the user to drive the robot around its environment. There will also be a demo window with instructions on how to interact with the demo and save the map when done exploring.

To begin the mapping demo:
 - In a new terminal source the setup file ($ source ./devel/setup.bash)
 - Launch the mapping_demo launch file 

```
roslaunch envi_nav mapping_demo.launch record_bag:=false
```

To close the mapping demo press ctrl^C in the terminal window twice (once to close the keyboard teleop and another to exit the launch file).

### Navigation Demo

The navigation demo is the second step in the envi_nav module. Once the robot has made a map of its environment it will have the ability to navigate the map it created through local and global path planning techniques. A custom global planner was created for this module which utilizes the RRT algorithm. This was chosen due to its speed and simplicity, since not much complexity is needed to navigate an apartment like environment. The user will interact with the robot through rviz to send it localization and navigation commands.

The navigation demo will launch the turtlebot in the custom gazebo world which simulates the apartment environemt. It will also launch rviz, like previously, to use for visualization, but this time a demo built map will be loaded into rviz since the robot has already discovered its world. The amcl node will be launched to allow for localization of the robot, and a revised move_base node which will utilize the custom RRT planner to generate a global path plan for the robot and send velocity command to the bot to reach the goal location.

To begin the navigation demo:
 - In a new terminal source the setup file ($ source ./devel/setup.bash)
 - Launch the navigation_demo launch file 

```
roslaunch envi_nav navigation_demo.launch record_bag:=false
```

To close the navigation demo press ctrl^C in the terminal window to exit the launch file.

## Bag File

To record a bag file set record\_bag = true when any launch file is ran. The bagfile will record for a maximum of 30 seconds.

To examine the recorded bag file run the command shown below:
```
rosbag info ~/.ros/<demo_name>.bag
```

To play the bag file run:
```
rosbag play ~/.ros/<demo_name>.bag
```

Where <demo_name> is either 'navigation_demo' or 'mapping_demo'.

## ROSTEST

To run the integration tests using rostest and gtest, in your catkin workspace run:

```
catkin_make run_tests envi_nav
```
In the summary section in the console window it will show you the number of tests, and whether they successfully passed.

## License

BSD 3-Clause (See LICENSE file for details)

## Future Development

 - Learned location names which will allow navigation by voice command
 - Improvements to RRT algorithm to speed up processing time and optimization of path
 - Implementation on a walking robot which will better simulate the pet-likeness of the RoboPet

## Developer Bio

  The developer of this module and product is Samantha Johnson, an engineer from Columbia, Maryland. Sam attended the University of Maryland for her B.S. which she received in 2014 with a major in Aerospace Engineering and a minor in Astronomy. She is back at the University of Maryland part-time to earn her M.Eng. in Robotics, while also working full-time as an aerospace engineer. Sam works for a small aerospace engineering company where she is on a task contracted with NASA. Based at the NASA Goddard Space Center, she works as a software developer and real-time operations analyst for NASA's Robotic Conjunction Assessment Risk Analysis (CARA) Team, where they work to keep space safe and debris-less by analyzing orbit data to determine if there are hazards along spacecraft flight paths. She also works as a developer on software releases that are critical to the work of the operators, and works to fix bugs and improve features in the production environment. Sam has always been fascinated by space and hopes to one day work in robotic space exploration, since the discovery of the "final frontier" has always been her passion. Even with a desire to work in the space industry, mental health has always been an issue close to home for her, so the idea for this product, the Mental Health RoboPet Companion came about when thinking about how deeply mental health affects the population and how hard it is to successfully treat. Sam hopes that whatever path her career takes her, she can make a positive impact and lasting impression on the world (or worlds!) for years to come.
