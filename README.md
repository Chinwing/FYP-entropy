# Final Year Project

## Overview

This work is the simulation for my final year project, which will runs a TurtleBot3 on a track in gazebo simulation using a joystick. 
This simulation implemented package from the Fessionia project as the entropy calculation
It aims to evaluate the effectivness of entropy metrics by comparing it with other cognitive load measurements


## Installation
It can be downloaded by coning the following git repository:

```bash
git clone https://github.com/Chinwing/FYP-entropy.git
git clone https://github.com/Chinwing/Turtlebot3_FYP.git
```
move them into a file in src of your catkin_ws, 
run cakin_make to build the workspace  

filepaths may need to be ammended in launch file

## Dependencies:
A Ubuntu machine (version 20.4) and a working ROS noetic environment is neeeded to run the code. To install all the dependencies go to the top directory of catkin_ws and run

```bash
rosdep install --from-paths src --ignore-src -r -y
```

execute the simulation

```bash
roslaunch behaviour_detection_package turtlebot3_entropy.launch
```

## References

Fessonia project - https://github.com/uob-erl/hrt_entropy/tree/smc21 

