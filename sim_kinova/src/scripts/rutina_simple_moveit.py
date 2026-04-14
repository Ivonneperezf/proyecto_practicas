#!/usr/bin/env python3

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi, fabs, cos, sqrt
from moveit_commander.conversions import pose_to_list
import numpy as np

"""==================================================================
            FUNCIONES GLOBALES PARA CONTROL GENERAL
====================================================================="""
# Funcion para calcular distancia
def dist(p, q):
    return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))

# Funcion para verificar la tolerancia de la posicion actual con respecto a la deseada
def all_close(goal, actual, tolerance):
    """
    Convenience method for testing if the values in two lists are within a tolerance of each other.
    For Pose and PoseStamped inputs, the angle between the two quaternions is compared (the angle
    between the identical orientations q and -q is calculated correctly).
    @param: goal       A list of floats, a Pose or a PoseStamped
    @param: actual     A list of floats, a Pose or a PoseStamped
    @param: tolerance  A float
    @returns: bool
    """
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(actual)
        x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(goal)
        # Euclidean distance
        d = dist((x1, y1, z1), (x0, y0, z0))
        # phi = angle between orientations
        cos_phi_half = fabs(qx0 * qx1 + qy0 * qy1 + qz0 * qz1 + qw0 * qw1)
        return d <= tolerance and cos_phi_half >= cos(tolerance / 2.0)

    return True

"""==================================================================
            CLASE PARA CONTROLAR EL BRAZO CON MOVEIT
====================================================================="""
class MoveGroupController(object):

    def __init__(self, home_joint_goal):
        super(MoveGroupController, self).__init__()
        # Inicializar ROS y MoveIt
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("kinova_moveit_tutorial", anonymous=True)
        # Información del robot
        self.robot = moveit_commander.RobotCommander()
        # Escena de planificación
        self.scene = moveit_commander.PlanningSceneInterface()
        group_name = "arm"
        self.move_group = moveit_commander.MoveGroupCommander(group_name)
        # Publicador para visualizar trayectorias en RViz
        self.display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )

        # ── Configuración del planificador ──────────────────────────────
        self.move_group.set_planning_time(10.0)          # más tiempo para encontrar plan
        self.move_group.set_num_planning_attempts(5)     # reintentos si falla
        self.move_group.set_max_velocity_scaling_factor(0.3)
        self.move_group.set_max_acceleration_scaling_factor(0.2)

        # ── Workspace cartesiano del Kinova m1n6s300 ────────────────────
        # Ajusta estos valores a tu celda real
        self.move_group.set_workspace([-1.0, -1.0, -0.2,
                                        1.0,  1.0,  1.5])

        # Informacion basica
        self.planning_frame = self.move_group.get_planning_frame()
        print("Frame de referencia:", self.planning_frame)

        self.eef_link = self.move_group.get_end_effector_link()
        print("End effector:", self.eef_link)

        self.group_names = self.robot.get_group_names()
        print("Grupos disponibles:", self.group_names)

        print("Estado actual del robot:")
        print(self.robot.get_current_state())
        self.home_joint_goal = home_joint_goal

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

    def go_to_pose_goal(self, pose_goal):
        self.move_group.set_pose_target(pose_goal)
        # Ejecutar movimiento y capturar resultado
        success = self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()
        if not success:
            rospy.logwarn("go_to_pose_goal: el planificador no encontró solución")
            return False
        current_pose = self.move_group.get_current_pose().pose
        return all_close(pose_goal, current_pose, 0.01)


def main():
    try:
        # joint_1 = 3.091497
        # joint_2 = 3.878493
        # joint_3 = 2.984441
        # joint_4 = -1.445133
        # joint_5 = 3.141593
        # joint_6 = -2.415254
        joint_1 = -3.1917
        joint_2 = 3.8806
        joint_3 = 2.9837
        joint_4 = -1.4455
        joint_5 = 3.1411
        joint_6 = -2.4153
        Home_pose = [joint_1, joint_2, joint_3, joint_4, joint_5, joint_6]
        print("*****************************\n\t\tIniciando nodo\n*****************************")
        input("*****************************\n\t\tPulsa ENTER para iniciar\n*****************************")
        move = MoveGroupController(home_joint_goal=Home_pose)
        input("*****************************\n\t\tPulsa ENTER para ir a HOME\n*****************************")
        ok = move.go_to_home()
        print(f"HOME {'OK' if ok else 'FALLIDO'}")

        # input("*****************************\n\t\tPulsa ENTER para ir a POSE 1\n*****************************")

        # # ── Quaternion: asegúrate del orden x, y, z, w ──────────────────
        # q = np.array([0.932, 0.181, -0.101, 0.298])  # orden: x, y, z, w
        # q = q / np.linalg.norm(q)                    # normalizar

        # pose_goal = geometry_msgs.msg.Pose(
        #     position=geometry_msgs.msg.Point(x=0.450, y=-0.140, z=0.305),
        #     orientation=geometry_msgs.msg.Quaternion(
        #         x=q[0], y=q[1], z=q[2], w=q[3]
        #     )
        # )
        print("*****************************\n\t\tMovimiento a HOME completado\n*****************************")
        # ok = move.go_to_pose_goal(pose_goal)          # ← llamada que faltaba
        # print(f"POSE 1 {'OK' if ok else 'FALLIDO'}")

    except rospy.ROSInterruptException:
        print("La ejecucion ha sido interrumpida por ROS.")
        return
    except KeyboardInterrupt:
        print("La ejecucion ha sido interrumpida por el usuario.")
        return

if __name__ == "__main__":
    main()