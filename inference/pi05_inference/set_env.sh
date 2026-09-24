#! /bin/bash

export ROS_DOMAIN_ID=38
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/zme/robot_config/system_config/zme_robot/cyclonedds_config/cyclonedds.xml

source /home/zme/ros2_humble_release/install/setup.bash
source /home/zme/robot/system/zme_robot/install/setup.bash
source /home/zme/lite_arm_manager/install/setup.bash