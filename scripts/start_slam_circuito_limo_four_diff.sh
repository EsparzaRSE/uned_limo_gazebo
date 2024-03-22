#! /usr/bin/env bash

ros2 launch uned_limo_gazebo circuito_limo_four_diff.launch.py &

sleep 5

ros2 launch uned_limo_gazebo limo_slam_toolbox.launch.py