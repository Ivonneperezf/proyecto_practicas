#!/usr/bin/env python3
import rospy
import moveit_commander
import sys

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node("check_limits")

robot = moveit_commander.RobotCommander()
arm = moveit_commander.MoveGroupCommander("arm")

# Imprime todos los joints y sus límites
for joint_name in arm.get_active_joints():
    joint = robot.get_joint(joint_name)
    bounds = joint.bounds()
    print(f"{joint_name}: min={bounds[0]:.4f}  max={bounds[1]:.4f}")