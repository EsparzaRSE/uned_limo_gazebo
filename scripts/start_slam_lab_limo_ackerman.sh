#! /usr/bin/env bash

ros2 launch uned_limo_gazebo laboratorio_limo_ackerman.launch.py &

sleep 5

ros2 launch uned_limo_gazebo limo_slam_toolbox.launch.py