#!/usr/bin/env python3
import sys
import rospy
import moveit_commander

def mover_gripper():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('control_dedos_kinova', anonymous=True)

    try:
        gripper_group = moveit_commander.MoveGroupCommander("gripper")
        gripper_group.set_goal_joint_tolerance(0.05) 
        gripper_group.set_planning_time(5.0)
    except Exception as e:
        rospy.logerr(f"No se encontró el grupo: {e}")
        return

    rospy.loginfo("--- Iniciando secuencia ---")

    rospy.loginfo("Abriendo dedos...")
    # Sincronizamos MoveIt con la posición real actual de Gazebo
    gripper_group.set_start_state_to_current_state()
    
    # Usamos valores un poquito menos extremos (0.01 en lugar de 0) 
    # para evitar chocar con los límites físicos del simulador
    gripper_group.go([0.01, 0.01, 0.01], wait=True)
    gripper_group.stop()
    
    rospy.sleep(2.0)

    # --- CERRAR ---
    rospy.loginfo("Cerrando dedos...")
    # Volvemos a sincronizar para evitar el error de "start point deviates"
    gripper_group.set_start_state_to_current_state()
    
    # Intenta con 0.8 en lugar de 1.0 si ves que rebota mucho
    gripper_group.go([0.8, 0.8, 0.8], wait=True)
    gripper_group.stop()

    rospy.loginfo("Secuencia completada.")
    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    mover_gripper()