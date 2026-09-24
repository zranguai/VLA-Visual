std_msgs/Header header
arm_interfaces/JointStatus[7] left_arm
arm_interfaces/JointStatus[7] right_arm
arm_interfaces/JointStatus[7] gripper
geometry_msgs/Pose left_arm_pose
geometry_msgs/Pose right_arm_pose
uint8[20] other_status

# other_status
# 0: traj_follow_primary_mode
# 1: traj_follow_secondary_mode
# 2: driver_status
# 3: gripper_config_type
