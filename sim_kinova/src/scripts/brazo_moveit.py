#!/usr/bin/env python3

from __future__ import print_function
from six.moves import input

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

from math import pi, fabs, cos, sqrt

# Función para calcular distancia
def dist(p, q):
    return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))

from moveit_commander.conversions import pose_to_list


def all_close(goal, actual, tolerance):
    """
    Verifica si dos valores (listas o poses) son similares dentro de una tolerancia.
    """
    if type(goal) is list:
        for i in range(len(goal)):
            if abs(actual[i] - goal[i]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.Pose:
        x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(actual)
        x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(goal)

        d = dist((x1, y1, z1), (x0, y0, z0))
        cos_phi_half = fabs(qx0*qx1 + qy0*qy1 + qz0*qz1 + qw0*qw1)

        return d <= tolerance and cos_phi_half >= cos(tolerance/2.0)

    return True


class MoveGroupPythonInterfaceTutorial(object):

    def __init__(self):
        super(MoveGroupPythonInterfaceTutorial, self).__init__()

        # 🔵 Inicializar ROS y MoveIt
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("kinova_moveit_tutorial", anonymous=True)

        # 🔵 Información del robot
        self.robot = moveit_commander.RobotCommander()

        # 🔵 Escena de planificación
        self.scene = moveit_commander.PlanningSceneInterface()

        group_name = "arm"
        self.move_group = moveit_commander.MoveGroupCommander(group_name)

        # Publicador para visualizar trayectorias en RViz
        self.display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )

        # Información básica
        self.planning_frame = self.move_group.get_planning_frame()
        print("Frame de referencia:", self.planning_frame)

        self.eef_link = self.move_group.get_end_effector_link()
        print("End effector:", self.eef_link)

        self.group_names = self.robot.get_group_names()
        print("Grupos disponibles:", self.group_names)

        print("Estado actual del robot:")
        print(self.robot.get_current_state())

        self.box_name = ""

    # 🟢 Movimiento por ángulos de articulación
    def go_to_joint_state(self):

        move_group = self.move_group

        # Obtener valores actuales
        joint_goal = move_group.get_current_joint_values()

        # Posición actual:  [0.0,  2.9,  1.3,  -2.07, -4.88, 0.0]
        # Límites joint_1:  [-3.14, 3.14]  → movemos +0.3
        # Límites joint_2:  [0.87,  5.41]  → movemos +0.3
        # Límites joint_3:  [0.61,  5.67]  → movemos +0.3
        # Límites joint_4:  [-3.14, 3.14]  → movemos +0.3
        # Límites joint_5:  [-3.14, 3.14]  → movemos +0.3
        # Límites joint_6:  [-3.14, 3.14]  → movemos +0.3

        joint_goal[0] =  0.3    # era 0.0
        joint_goal[1] =  3.2    # era 2.9
        joint_goal[2] =  1.6    # era 1.3
        joint_goal[3] = -1.77   # era -2.07
        joint_goal[4] = -4.58   # era -4.88
        joint_goal[5] =  0.3    # era 0.0

        # Ejecutar movimiento
        move_group.go(joint_goal, wait=True)
        move_group.stop()

        current_joints = move_group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)

    # 🟢 Movimiento a una pose (XYZ + orientación)
    def go_to_pose_goal(self):
        move_group = self.move_group

        move_group.set_planning_time(15)
        move_group.set_num_planning_attempts(20)

        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.x =  0.960
        pose_goal.orientation.y =  0.256
        pose_goal.orientation.z =  0.058
        pose_goal.orientation.w =  0.099

        pose_goal.position.x =  0.434
        pose_goal.position.y = -0.030
        pose_goal.position.z =  0.370

        move_group.set_pose_target(pose_goal)

        # Separar plan y ejecucion
        plan = move_group.plan()
        success = plan[0]

        if success:
            print("Plan encontrado...")
            # Esperar un momento antes de ejecutar
            rospy.sleep(1.0)
            move_group.execute(plan[1], wait=True)
            rospy.sleep(1.0)
        else:
            print("No se encontro plan")

        move_group.stop()
        move_group.clear_pose_targets()
        print(f"Exito: {success}")
        current_pose = move_group.get_current_pose().pose
        return all_close(pose_goal, current_pose, 0.01)

    #  Trayectoria cartesiana
    def plan_cartesian_path(self, scale=1):

        move_group = self.move_group

        waypoints = []

        wpose = move_group.get_current_pose().pose

        # Movimiento en Z
        wpose.position.z -= scale * 0.1
        waypoints.append(copy.deepcopy(wpose))

        # Movimiento en X
        wpose.position.x += scale * 0.1
        waypoints.append(copy.deepcopy(wpose))

        # Movimiento en Y
        wpose.position.y += scale * 0.1
        waypoints.append(copy.deepcopy(wpose))

        # Planificación
        plan, fraction = move_group.compute_cartesian_path(
            waypoints, 0.01
        )

        return plan, fraction

    # 🟢 Mostrar trayectoria en RViz
    def display_trajectory(self, plan):

        display_trajectory = moveit_msgs.msg.DisplayTrajectory()

        display_trajectory.trajectory_start = self.robot.get_current_state()
        display_trajectory.trajectory.append(plan)

        self.display_trajectory_publisher.publish(display_trajectory)

    # 🟢 Ejecutar trayectoria
    def execute_plan(self, plan):

        self.move_group.execute(plan, wait=True)

    # 🟢 Agregar objeto (caja)
    def add_box(self, timeout=4):

        box_pose = geometry_msgs.msg.PoseStamped()

        # 🔴 CAMBIO: frame del efector de Kinova
        box_pose.header.frame_id = self.eef_link

        box_pose.pose.orientation.w = 1.0
        box_pose.pose.position.z = 0.1

        self.box_name = "box"

        self.scene.add_box(self.box_name, box_pose, size=(0.05, 0.05, 0.05))

        rospy.sleep(1)
        return True

    # 🟢 Adjuntar objeto al robot
    def attach_box(self):

        touch_links = self.robot.get_link_names(group="arm")

        self.scene.attach_box(self.eef_link, self.box_name, touch_links=touch_links)

        rospy.sleep(1)
        return True

    # 🟢 Separar objeto
    def detach_box(self):

        self.scene.remove_attached_object(self.eef_link, name=self.box_name)

        rospy.sleep(1)
        return True

    # 🟢 Eliminar objeto
    def remove_box(self):

        self.scene.remove_world_object(self.box_name)

        rospy.sleep(1)
        return True


def main():
    try:
        print("===== Tutorial MoveIt con Kinova =====")

        input("Enter para iniciar...")
        tutorial = MoveGroupPythonInterfaceTutorial()

        #input("Mover por articulaciones...")
        #tutorial.go_to_joint_state()

        input("Mover a pose...")
        tutorial.go_to_pose_goal()

        # input("Plan cartesiano...")
        # plan, _ = tutorial.plan_cartesian_path()

        # input("Mostrar trayectoria...")
        # tutorial.display_trajectory(plan)

        # input("Ejecutar trayectoria...")
        # tutorial.execute_plan(plan)

        # input("Agregar caja...")
        # tutorial.add_box()

        # input("Adjuntar caja...")
        # tutorial.attach_box()

        # input("Desacoplar caja...")
        # tutorial.detach_box()

        # input("Eliminar caja...")
        # tutorial.remove_box()

        print("✅ Demo completado")

    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()