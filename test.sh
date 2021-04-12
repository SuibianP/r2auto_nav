#!/bin/bash

# quick and dirty test shell script for EG2310

trap 'pkill -f cartographer; pkill -f gazebo; pkill -f turtlebot; pkill -f ros; pkill -f rviz; pkill -f gzclient; pkill -f occupancy;' EXIT INT HUP
. /opt/ros/foxy/setup.bash
ros2 launch -a turtlebot3_gazebo turtlebot3_world.launch.py &> /dev/null &
ros2 launch -a turtlebot3_cartographer cartographer.launch.py use_sim_time:=True &> /dev/null &
python3 nav.py 2>&1 | tee output.log
