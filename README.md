# EnviNav
The Navigation Module for Mental Health RoboPet Companion!

ACME Robotics is developing an in home mental health robotic pet companion. This companion is geared toward individuals with moderate-severe mental health conditions that would benefit from constant supervision and stimulation from an external source. The target consumer is a young to middle aged adult, who may live alone or are in a situation where there is limited supervision, and who also suffers from a behavioral disorder such as depression, anxiety, or bipolar disorder. The goal of this product is to provide companionship, motivation, emotional stabilization, activity notifications, and emergency resources to aid in the recovery process for an individual suffering from a mental health illness. This product hopes to speed up the recovery time and decrease the relapse rate for these individuals, and also reduce the high number of suicides that occur amongst this group.

The developed product will be a wheeled mobile pet-like robot that will be capable of interacting with its human companion through voice commands, wearable technology, and facial expressions and gestures. Previous development has occurred on the Mental State Reactionary Decision Maker (MSRDM) component of this robot and a demo was provided that displayed the robot’s ability to react to various states of human emotion as well as learn what type of interaction its human prefers. For this development cycle the focus will be on the navigation aspect of the RoboPet, which will be called EnviNav. This is a crucial component of the system because the robot needs to learn about its environment, which would be the home of the human, so that it can effectively assist the human as necessary. The robot will need to not only gather and store information about its environment, but also autonomously navigate to various locations, whether directed by voice command or by beacon, without colliding into known or unknown obstacles. The learning capabilities of the RoboPet will allow it to be tailored specifically to its human and its environment so that it can be integrated seamlessly into its human’s life.

EnviNav will initialize the robot by allowing it to discover its environment and record a base map. To do this the robot will use the SLAM method. The robot will be guided around the environment and allowed to record and save the map it creates using a laser sensor. Once the robot has discovered its environment it will be able to localize itself to allow it to travel to set locations on the map. The robot will need to be equipped with a global planner, which will allow the robot to plan a path to its destination, but will also need to be equipped with a local planner, which will allow the robot to avoid unknown obstacles that may appear along its path, and then successfully migrate back to the global path plan.

## Project Backlog and Sprint Backlog
All issues for the project backlog are being tracked in the issues section of this repo (https://github.com/sjohns09/EnviNav/issues).

The sprint backlog is located in the projects section where each sprint has its own project and issues tied to milestones which help track the due date for each sprint (https://github.com/sjohns09/EnviNav/projects).

## Worklog and Sprint Review Notes

The Log for this project is located at:
https://docs.google.com/a/terpmail.umd.edu/spreadsheets/d/1aHDHUWVx5oCtyFJUyxdDZbs8FFohRLv_YU-jJhAd0oE/edit?usp=sharing

The first tab of this sheet contains the Worklog, and the second tab contains the Review notes from the post sprint reviews.

## License

BSD 3-Clause (See LICENSE file for details)

## Developer Bio

  The developer of this module and product is Samantha Johnson, an engineer from Columbia, Maryland. Sam attended the University of Maryland for her B.S. which she recieved in 2014 with a major in Aerospace Engineering and a minor in Astronomy. She is back at the University of Maryland part-time to earn her M.Eng. in Robotics, while also working full-time as an aerospace engineer. Sam works for a small aerospace engineering company where she is on a task contracted with NASA. Based at the NASA Goddard campus, she works as a software developer and real-time operations analyst for NASA's Robotic Conjunction Assessment Risk Analysis (CARA) Team, where they work to keep space safe and debris-less by analyzing orbit data to determine if there are hazards along spacecraft flight paths. She also works as a developer on software releases that are critical to the work of the operators, and works to fix bugs and improve features in the production environment. Sam has always been fascinated by space and hopes to one day work in robotic space exploration, since the discovery of the "final frontier" has always been her passion. Even with a desire to work in the space industry, mental health has always been an issue close to home for her, so the idea for this product, the Mental Health RoboPet Companion came about when thinking about how deeply mental health affects the population and how hard it is to successfully treat. Sam hopes that whatever path her career takes her, she can make a positive impact and lasting impression on the world (or worlds!).
