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

    def __init__(self):
        super(MoveGroupController, self).__init__()
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("kinova_moveit_tutorial", anonymous=True)

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
        self.move_group.set_max_velocity_scaling_factor(0.3)
        self.move_group.set_max_acceleration_scaling_factor(0.2)
        self.move_group.set_workspace([-1.0, -1.0, -0.2,
                                        1.0,  1.0,  1.5])

        self.planning_frame = self.move_group.get_planning_frame()
        self.eef_link       = self.move_group.get_end_effector_link()
        self.group_names    = self.robot.get_group_names()

        print("Frame de referencia:", self.planning_frame)
        print("End effector:",        self.eef_link)
        print("Grupos disponibles:",  self.group_names)
        print("Estado actual del robot:")
        print(self.robot.get_current_state())

        # ============================================================
        # Estado interno: guardamos el último punto recibido y un flag
        # para no ejecutar movimientos en paralelo si llega otro punto
        # mientras el brazo todavía está en movimiento.
        # ============================================================
        self.target_point  = None   # último PointStamped recibido
        self.moving        = False  # True mientras MoveIt ejecuta

        # ============================================================
        # Suscripción al transformer. Cada vez que llega un punto
        # transformado a la base del robot, se llama a point_callback.
        # ============================================================
        rospy.Subscriber('/object_centroid_robot', PointStamped, self.point_callback)
        rospy.loginfo("Mover listo. Esperando coordenadas en /object_centroid_robot ...")

    # ================================================================
    # Callback: se ejecuta cada vez que el transformer publica un punto.
    # Solo guardamos el punto más reciente; el movimiento lo ejecuta
    # el bucle principal para no bloquear el hilo de ROS.
    # ================================================================
    def point_callback(self, msg):
        if not self.moving:
            self.target_point = msg

    # ================================================================
    # Construye un Pose a partir del PointStamped recibido.
    # X e Y vienen del transformer (posición del objeto en la base).
    # Z se toma de la posición ACTUAL del end effector para no mover
    # en altura mientras la profundidad no esté integrada.
    # La orientación se mantiene igual a la actual para no rotar.
    # ================================================================
    def build_pose_from_point(self, point_msg):
        # Pose actual del end effector como referencia
        current_pose = self.move_group.get_current_pose().pose

        pose_goal = geometry_msgs.msg.Pose()

        # X e Y del objeto detectado (en metros respecto a la base)
        pose_goal.position.x = point_msg.point.x
        pose_goal.position.y = point_msg.point.y
        OFFSET_AGARRE = 0.05  # 30 cm sobre el objeto
        # Z fijo: mantenemos la altura actual del end effector
        # hasta integrar la profundidad con nube de puntos
        pose_goal.position.z = point_msg.point.z + OFFSET_AGARRE

        # Orientación igual a la actual: no rotamos el end effector
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

    # ================================================================
    # Bucle principal: revisa si hay un punto nuevo pendiente y lo
    # ejecuta. Al terminar libera el flag para aceptar el siguiente.
    # ================================================================
    def run(self):
        rate = rospy.Rate(10)  # 10 Hz, revisa si hay punto nuevo
        while not rospy.is_shutdown():
            if self.target_point is not None and not self.moving:
                self.moving       = True
                point             = self.target_point
                self.target_point = None  # consumimos el punto

                rospy.loginfo(
                    f"Moviendo a → X:{point.point.x:.3f}  Y:{point.point.y:.3f}"
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
        controller = MoveGroupController()
        controller.run()
    except rospy.ROSInterruptException:
        pass