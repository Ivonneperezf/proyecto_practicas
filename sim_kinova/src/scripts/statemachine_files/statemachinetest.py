#!/usr/bin/env python

# POR EL MOMENTO LA MAQUINA DE ESTADOS SE QUEDA HASTA AQUI

import rospy
import smach
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float64MultiArray, String

# Estado HOME
class Home(smach.State):
    def __init__(self):
        # Transiciones posibles -> Done: movimiento completado, Failed: movimiento fallido
        smach.State.__init__(self, outcomes=['Done', 'Failed']) # PERSONAL: DEFINIR QUE HACER EN CASO DE FALLO

        # Publicación de tópicos
        self.cartesian_point = rospy.Publisher('/cartesian_goal', PointStamped, queue_size=10)
        self.joint_position = rospy.Publisher('/joint_goal', Float64MultiArray, queue_size=10)

        # Definicion de pose HOME cartesiana
        self.home_pose = PointStamped()
        self.home_pose.point.x = 0.434
        self.home_pose.point.y = -0.002
        self.home_pose.point.z = 0.362

        # Definición de pose HOME articular (en radianes)
        self.home_joint_goal = Float64MultiArray()
        self.home_joint_goal.data = [-3.1917, 3.8806, 2.9837, -1.4455, 3.1411, -2.4153]

    def execute(self, userdata):
        rospy.loginfo("Ejecutando estado: HOME")

        # Comentar y descomentar en caso de que se use uno u otro tipo de movimiento (cartesiano o articular)
        self.cartesian_point.publish(self.home_pose)
        #self.joint_position.publish(self.home_joint_goal)
        status_msg = rospy.wait_for_message('/motion_done', String)
        if status_msg.data == "DONE":
            return 'Done'
        else:
            return 'Failed'

# Estado ESPERAR_PUNTO
class Esperar_Punto(smach.State):
    def __init__ (self):
        smach.State.__init__(self, outcomes=['received_point'],
                             input_keys=['point_received']) 
    
    def execute(self, userdata):
        rospy.loginfo("Ejecutando estado: ESPERAR_PUNTO")
        point_robot = rospy.wait_for_message('/object_centroid_robot', PointStamped)
        userdata.point_received = point_robot
        return 'received_point'
    
class Mover_A_Punto(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Done', 'Failed'],
                             input_keys=['point_to_move']) 
        # Nodo publicador en el tópico de movimiento
        self.cartesian_point = rospy.Publisher('/cartesian_goal', PointStamped, queue_size=10)
        
        def execute(self, userdata):
            rospy.login("Ejecutando estado: MOVER_A_PUNTO")
            # Se envia el punto recibido del centro del objeto al nodo de movimiento
            self.cartesian_point.publish(userdata.point_to_move)

            # Esoeramos a que se reciba la confirmacion de movimiento
            status_msg = rospy.wait_for_message('/motion_done', String)
            if status_msg.data == "DONE":
                return 'Done'
            else:
                return 'Failed'


