#!/usr/bin/env python3
import rospy
import moveit_commander
import sys
import numpy as np

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node("check_workspace")

arm = moveit_commander.MoveGroupCommander("arm")
arm.set_planning_time(0.5)  # poco tiempo por punto para ir rapido

print("Probando puntos alcanzables... espera un momento\n")

x_range = np.arange(-0.9, 0.9, 0.15)
y_range = np.arange(-0.9, 0.9, 0.15)
z_range = np.arange(0.0, 1.2, 0.15)

x_min, x_max = 999, -999
y_min, y_max = 999, -999
z_min, z_max = 999, -999

import geometry_msgs.msg
count = 0

for x in x_range:
    for y in y_range:
        for z in z_range:
            pose = geometry_msgs.msg.Pose()
            pose.position.x = x
            pose.position.y = y
            pose.position.z = z
            pose.orientation.w = 1.0

            arm.set_pose_target(pose)
            plan = arm.plan()

            # En noetic plan() devuelve tupla (success, plan, ...)
            success = plan[0] if isinstance(plan, tuple) else (len(plan.joint_trajectory.points) > 0)

            if success:
                count += 1
                x_min = min(x_min, x)
                x_max = max(x_max, x)
                y_min = min(y_min, y)
                y_max = max(y_max, y)
                z_min = min(z_min, z)
                z_max = max(z_max, z)

arm.clear_pose_targets()

print(f"Puntos alcanzables encontrados: {count}")
print(f"\nWorkspace aproximado:")
print(f"  X: {x_min:.2f} a {x_max:.2f} metros")
print(f"  Y: {y_min:.2f} a {y_max:.2f} metros")
print(f"  Z: {z_min:.2f} a {z_max:.2f} metros")