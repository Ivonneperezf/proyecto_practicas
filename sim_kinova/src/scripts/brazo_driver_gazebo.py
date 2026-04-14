#! /usr/bin/env python3

import rospy # libreria de ROS para Python
from trajectory_msgs.msg import JointTrajectory # mensaje para enviar trayectorias de articulaciones
from trajectory_msgs.msg import JointTrajectoryPoint # mensaje para definir un punto en la trayectoria de articulaciones
from std_srvs.srv import Empty # servicio vacío de ROS
import argparse # libreria para leer argumentos de la línea de comandos
import time # libreria para manejar el tiempo

# Funcion para parsear los argumentos de la línea de comandos
def argumentParser(argument):
  """ Argument parser """
  # Instancia de parser de argumentos
  parser = argparse.ArgumentParser(description='Drive robot joint to command position') 
  # Argumento kinova_robotType para definir el tipo de robot
  parser.add_argument('kinova_robotType', metavar='kinova_robotType', type=str, default='j2n6a300',
                      help='kinova_RobotType is in format of: [{j|m|r|c}{1|2}{s|n}{4|6|7}{s|a}{2|3}{0}{0}]. eg: j2n6a300 refers to jaco v2 6DOF assistive 3fingers. Please be noted that not all options are valided for different robot types.')
  # Parsear argumentos de linea de comandos (Ejecutado con pyhton3)
  #args_ = parser.parse_args(argument) 
  # Parsear argumentos ejecutado con rosrun
  argv = rospy.myargv()
  args_ = parser.parse_args(argv[1:])
  # Extraemos el prefix usando el argumento leido para definir el espacio de nombres del robot
  prefix = args_.kinova_robotType
  # Caracter 3 de prefix es el numero de articulaciones
  nbJoints = int(args_.kinova_robotType[3])	
  # Caracter 5 de prefix es el numero de dedos
  nbfingers = int(args_.kinova_robotType[5])	
  # Retorna numero de articulaciones, dedos y prefix del robot
  return prefix, nbJoints, nbfingers

# Funcion para mover las articulaciones del robot a una posición dada
def moveJoint (jointcmds,prefix,nbJoints):
  # Definimos el nombre del topico para publicar las trayectorias de articulaciones
  topic_name = '/' + prefix + '/effort_joint_trajectory_controller/command'
  # Creamos un nodo publicador para enviar el mensaje definido en topic_name
  pub = rospy.Publisher(topic_name, JointTrajectory, queue_size=1)
  # Instanciamos un mensaje de tipo JointTrajectory
  jointCmd = JointTrajectory()  
  # Instanciamos un mensaje de tipo JointTrajectoryPoint 
  point = JointTrajectoryPoint()
  # Definimos el tiempo de inicio del mensaje como el tiempo actual
  jointCmd.header.stamp = rospy.Time.now() + rospy.Duration.from_sec(0.0);  
  # Definimos el tiempo para alcanzar la posición deseada como 5 segundos
  point.time_from_start = rospy.Duration.from_sec(5.0)
  # Iteramos de 0 al numero de articulaciones del robot para llenar el mensaje con los nombres de las articulaciones y las posiciones deseadas
  for i in range(0, nbJoints):
    # Agregamos el nombre de cada articulación al mensaje
    jointCmd.joint_names.append(prefix +'_joint_'+str(i+1))
    # Agregamos la posición deseada para cada articulación al mensaje
    point.positions.append(jointcmds[i])
    # Agregamos velocidad, aceleración y esfuerzo como 0 para cada articulación
    point.velocities.append(0)
    # Agregamos aceleración como 0 para cada articulación
    point.accelerations.append(0)
    # Agregamos esfuerzo como 0 para cada articulación
    point.effort.append(0) 
  # Agregamos el punto definido al mensaje de trayectoria de articulaciones
  jointCmd.points.append(point)
  # Definimos una tasa de publicación de 100 Hz
  rate = rospy.Rate(100)
  # Publicamos el mensaje de trayectoria de articulaciones 50 veces para asegurar que el robot reciba el comando
  count = 0
  while (count < 50):
    pub.publish(jointCmd)
    count = count + 1
    rate.sleep()   

# Funcion para mover los dedos del robot a una posición dada
def moveFingers (jointcmds,prefix,nbJoints):
  # La definicion de este movimiento es similar, salvo a algunas modificaciones
  topic_name = '/' + prefix + '/effort_finger_trajectory_controller/command'
  pub = rospy.Publisher(topic_name, JointTrajectory, queue_size=1)  
  jointCmd = JointTrajectory()  
  point = JointTrajectoryPoint()
  jointCmd.header.stamp = rospy.Time.now() + rospy.Duration.from_sec(0.0);  
  # El tiempo es despues de haber alcanzado la posicion de las articulaciones
  point.time_from_start = rospy.Duration.from_sec(5.0)
  for i in range(0, nbJoints):
    jointCmd.joint_names.append(prefix +'_joint_finger_'+str(i+1))
    point.positions.append(jointcmds[i])
    point.velocities.append(0)
    point.accelerations.append(0)
    point.effort.append(0) 
  jointCmd.points.append(point)
  rate = rospy.Rate(100)
  # El mensaje se publica esta vez 500 veces para asegurar que el robot reciba el comando de movimiento de los dedos
  count = 0
  while (count < 500):
    pub.publish(jointCmd)
    count = count + 1
    rate.sleep()  


if __name__ == '__main__':
  try:    
    # Inicializamos el nodo de ROS con el nombre 'move_robot_using_trajectory_msg'
    rospy.init_node('move_robot_using_trajectory_msg')	
    # Llamamos a la funcion argumentParser para leer la linea de comandos	
    prefix, nbJoints, nbfingers = argumentParser(None)    
    #allow gazebo to launch
    time.sleep(5)

    # Unpause the physics
    rospy.wait_for_service('/gazebo/unpause_physics')
    unpause_gazebo = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
    resp = unpause_gazebo()

    if (nbJoints==6):
      #home robots
      moveJoint ([0.0,2.9,1.3,4.2,1.4,0.0],prefix,nbJoints)
    else:
      moveJoint ([0.0,2.9,0.0,1.3,4.2,1.4,0.0],prefix,nbJoints)

    moveFingers ([1,1,1],prefix,nbfingers)
  except rospy.ROSInterruptException:
    print("program interrupted before completion")