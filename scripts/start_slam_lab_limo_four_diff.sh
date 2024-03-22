#! /usr/bin/env bash

#. ~/.bashrc
#. $HOME/uned_tfg/ros2_ws/install/setup.bash

ros2 launch uned_limo_gazebo laboratorio_limo_four_diff.launch.py &

sleep 5

ros2 launch uned_limo_gazebo limo_slam_toolbox.launch.py