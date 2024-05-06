#!/bin/bash

# configure gazebo
echo 'export GZ_SIM_SYSTEM_PLUGIN_PATH=/workspace/ardupilot_gazebo_ws/src/ardupilot_gazebo/build:${GZ_SIM_SYSTEM_PLUGIN_PATH}' >> ~/.bashrc
echo 'export GZ_SIM_RESOURCE_PATH=/workspace/ardupilot_gazebo_ws/src/ardupilot_gazebo/models:/workspace/ardupilot_gazebo_ws/src/ardupilot_gazebo/worlds:${GZ_SIM_RESOURCE_PATH}' >> ~/.bashrc
# configure ardupilot sitl
echo 'export PATH=$PATH:/workspace/ardupilot_sitl/src/ardupilot/Tools/autotest' >> ~/.bashrc 
echo 'export PATH=/usr/lib/ccache:$PATH' >> ~/.bashrc 