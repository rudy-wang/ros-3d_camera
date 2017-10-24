# -*- coding: utf-8 -*-
from __future__ import print_function
import argparse
import os
import stat
import sys

# find the import for catkin's python package - either from source space or from an installed underlay
if os.path.exists(os.path.join('/opt/ros/kinetic/share/catkin/cmake', 'catkinConfig.cmake.in')):
    sys.path.insert(0, os.path.join('/opt/ros/kinetic/share/catkin/cmake', '..', 'python'))
try:
    from catkin.environment_cache import generate_environment_script
except ImportError:
    # search for catkin package in all workspaces and prepend to path
    for workspace in "/home/ecs/catkin_ws/devel;/home/ecs/catkin_ws/devel_isolated/turtlebot_stdr;/home/ecs/catkin_ws/devel_isolated/turtlebot_stage;/home/ecs/catkin_ws/devel_isolated/turtlebot_simulator;/home/ecs/catkin_ws/devel_isolated/turtlebot_gazebo;/home/ecs/catkin_ws/devel_isolated/turtlebot3_teleop;/home/ecs/catkin_ws/devel_isolated/turtlebot3_slam;/home/ecs/catkin_ws/devel_isolated/turtlebot3_navigation;/home/ecs/catkin_ws/devel_isolated/turtlebot3_msgs;/home/ecs/catkin_ws/devel_isolated/turtlebot3_description;/home/ecs/catkin_ws/devel_isolated/turtlebot3_bringup;/home/ecs/catkin_ws/devel_isolated/turtlebot3;/home/ecs/catkin_ws/devel_isolated/nav2d_tutorials;/home/ecs/catkin_ws/devel_isolated/nav2d_remote;/home/ecs/catkin_ws/devel_isolated/nav2d_exploration;/home/ecs/catkin_ws/devel_isolated/nav2d_navigator;/home/ecs/catkin_ws/devel_isolated/nav2d_operator;/home/ecs/catkin_ws/devel_isolated/nav2d_karto;/home/ecs/catkin_ws/devel_isolated/nav2d_msgs;/home/ecs/catkin_ws/devel_isolated/nav2d_localizer;/home/ecs/catkin_ws/devel_isolated/nav2d;/home/ecs/catkin_ws/devel_isolated/cartographer_turtlebot;/home/ecs/catkin_ws/install_isolated;/opt/ros/kinetic".split(';'):
        python_path = os.path.join(workspace, 'lib/python2.7/dist-packages')
        if os.path.isdir(os.path.join(python_path, 'catkin')):
            sys.path.insert(0, python_path)
            break
    from catkin.environment_cache import generate_environment_script

code = generate_environment_script('/home/ecs/catkin_ws/src/cob_environment_perception/cob_3d_segmentation/build/devel/env.sh')

output_filename = '/home/ecs/catkin_ws/src/cob_environment_perception/cob_3d_segmentation/build/catkin_generated/setup_cached.sh'
with open(output_filename, 'w') as f:
    #print('Generate script for cached setup "%s"' % output_filename)
    f.write('\n'.join(code))

mode = os.stat(output_filename).st_mode
os.chmod(output_filename, mode | stat.S_IXUSR)
