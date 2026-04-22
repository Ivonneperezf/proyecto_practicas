#!/usr/bin/env python3

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi, fabs, cos, sqrt
from moveit_commander.conversions import pose_to_list
from geometry_msgs.msg import PointStamped
import numpy as np

"""==================================================================
            FUNCIONES GLOBALES PARA CONTROL GENERAL
====================================================================="""
def dist(p, q):
    return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))

def all_close(goal, actual, tolerance):
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False
    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)
    elif type(goal) is geometry_msgs.msg.Pose:
        x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(actual)
        x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(goal)
        d = dist((x1, y1, z1), (x0, y0, z0))
        cos_phi_half = fabs(qx0 * qx1 + qy0 * qy1 + qz0 * qz1 + qw0 * qw1)
        return d <= tolerance and cos_phi_half >= cos(tolerance / 2.0)
    return True

"""==================================================================
            CLASE PARA CONTROLAR EL BRAZO CON MOVEIT
====================================================================="""
class MoveGroupController(object):

    def __init__(self,home_joint_goal):
        super(MoveGroupController, self).__init__()
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("kinova_moveit", anonymous=True)

        self.robot      = moveit_commander.RobotCommander()
        self.scene      = moveit_commander.PlanningSceneInterface()
        self.move_group = moveit_commander.MoveGroupCommander("arm")

        self.display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )

        self.move_group.set_planning_time(10.0)
        self.move_group.set_num_planning_attempts(5)
        self.move_group.set_max_velocity_scaling_factor(0.3) # Velocidad al 30% para movimientos más suaves
        self.move_group.set_max_acceleration_scaling_factor(0.2) # Aceleración al 20% para evitar movimientos bruscos
        self.move_group.set_workspace([-1.0, -1.0, -0.2, # Definición de tamaño de workspace
                                        1.0,  1.0,  1.5])

        self.planning_frame = self.move_group.get_planning_frame()
        self.eef_link       = self.move_group.get_end_effector_link()
        self.group_names    = self.robot.get_group_names()

        print("Frame de referencia:", self.planning_frame)
        print("End effector:",        self.eef_link)
        print("Grupos disponibles:",  self.group_names)
        print("Estado actual del robot:")
        print(self.robot.get_current_state())
        self.home_joint_goal = home_joint_goal

        self.target_point  = None   # ultimo PointStamped recibido
        self.moving        = False  
        
        rospy.Subscriber('/object_centroid_robot', PointStamped, self.point_callback)
        rospy.loginfo("Mover listo. Esperando coordenadas en /object_centroid_robot ...")

    def point_callback(self, msg):
        if not self.moving:
            self.target_point = msg

    # Construye un Pose a partir del PointStamped recibido.
    # X e Y vienen del transformer (posición del objeto en la base).
    # Z se toma de la posición ACTUAL del end effector para no mover
    # en altura mientras la profundidad no esté integrada.
    # La orientación se mantiene igual a la actual para no rotar.
    def build_pose_from_point(self, point_msg):
        # Pose actual del end effector como referencia
        current_pose = self.move_group.get_current_pose().pose

        pose_goal = geometry_msgs.msg.Pose()

        # X y Y del objeto detectado (en metros respecto a la base)
        pose_goal.position.x = point_msg.point.x
        pose_goal.position.y = point_msg.point.y
        OFFSET_AGARRE = 0.1  # 10 cm sobre el objeto
        # Z con offset
        pose_goal.position.z = point_msg.point.z + OFFSET_AGARRE

        # Orientación igual a la actual
        pose_goal.orientation = current_pose.orientation

        return pose_goal

    def go_to_pose_goal(self, pose_goal):
        self.move_group.set_pose_target(pose_goal)
        success = self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()
        if not success:
            rospy.logwarn("go_to_pose_goal: el planificador no encontró solución")
            return False
        current_pose = self.move_group.get_current_pose().pose
        return all_close(pose_goal, current_pose, 0.01)
    
    def go_to_home(self):
        joint_goal = self.move_group.get_current_joint_values()
        for i in range(len(self.home_joint_goal)):
            joint_goal[i] = self.home_joint_goal[i]
        # PARAMETROS PARA VELOCIDAD Y ACELERACION (ESCALA DE 0.0 A 1.0)
        # self.move_group.set_max_velocity_scaling_factor(0.3)
        # self.move_group.set_max_acceleration_scaling_factor(0.2)
        # Ejecutar movimiento
        success = self.move_group.go(joint_goal, wait=True)
        self.move_group.stop()
        if not success:
            rospy.logwarn("go_to_home: el planificador no encontró solución")
            return False
        current_joints = self.move_group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)

    # Bucle principal
    def run(self):
        # Va hacia pose HOME
        input("*****************************\n\t\tPulsa ENTER para ir a HOME\n*****************************")
        ok = self.go_to_home()
        print(f"HOME {'OK' if ok else 'FALLIDO'}")
        print("*****************************\n\t\tMovimiento a HOME completado\n*****************************")

        rospy.loginfo_throttle(2.0, f"Loop activo | target_point={self.target_point} | moving={self.moving}")

        rate = rospy.Rate(10)  # 10 Hz, revisa si hay punto nuevo
        while not rospy.is_shutdown():
            if self.target_point is not None and not self.moving:
                self.moving       = True
                point             = self.target_point
                self.target_point = None  # consumimos el punto

                rospy.loginfo(
                    f"Moviendo a -> X:{point.point.x:.3f}  Y:{point.point.y:.3f}  Z:{(point.point.z+self.OFFSET_AGARRE):.3f}"
                )

                pose_goal = self.build_pose_from_point(point)
                success   = self.go_to_pose_goal(pose_goal)

                if success:
                    rospy.loginfo("Movimiento completado correctamente.")
                else:
                    rospy.logwarn("Movimiento fallido, esperando siguiente punto.")

                self.moving = False

            rate.sleep()

if __name__ == '__main__':
    try:
        joint_1 = -3.1917
        joint_2 = 3.8806
        joint_3 = 2.9837
        joint_4 = -1.4455
        joint_5 = 3.1411
        joint_6 = -2.4153
        Home_pose = [joint_1, joint_2, joint_3, joint_4, joint_5, joint_6]
        controller = MoveGroupController(home_joint_goal=Home_pose)
        controller.run()
    except rospy.ROSInterruptException:
        pass