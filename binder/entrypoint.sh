#!/bin/bash

# Launch the ROS core and web tools when container starts
source ${HOME}/workspace/ros/devel/setup.bash
roscore &
roslaunch --wait giskardpy_ros giskardpy_tracy_standalone.launch &
roslaunch --wait rvizweb rvizweb.launch config_file:=${HOME}/binder/standalone_tracy.rviz &

# The following line will allow the binderhub start Jupyterlab, should be at the end of the entrypoint.
exec "$@"
