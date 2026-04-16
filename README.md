# Proyecto de practicas profesionales Primavera 2026

Estos son paquetes creados para ejecutar movimiento y modificar cosas, sin embargo, existen algunas modificaciones que deben de hacerse en el repositorio clonado propio del brazo, proveniente del repositorio oficial de [kinova-ros](https://github.com/kinovarobotics/kinova-ros).

Algunas de las modificaciones más importantes son las modificaciones realizadas para habilitar correctamente los controladores adecuados para [Moveit!](https://moveit.github.io/moveit_tutorials/), ya que existieron problemas puntuales sobre su instalación.

Para navegar fácilmente en el repositorio es conveniente, después de hacer source en el workspace, ejecutar el siguiente comando:

```bash
  roscd m1n6s300_moveit_config/launch
```

En el archivo ***m1n6s300_gazebo_demo.launch*** debemos agregar algunas líneas de código, ya que los controladores cargados para la simulación son los controladore usados por el brazo físico, a diferencia de los controladores necesarios para gazebo.


```XML
  <!-- En los argumentos agregar -->
  <arg name="gazebo_sim" default="true"/>
```

El bloque siguiente debe agregarse el argumento anterior

```XML
<!-- Se agrega la parte de gazebo_sim -->
<include file="$(find m1n6s300_moveit_config)/launch/move_group_m1n6s300.launch">
  <arg name="allow_trajectory_execution" value="true"/>
  <arg name="fake_execution" value="false"/>
  <arg name="info" value="true"/>
  <arg name="debug" value="$(arg debug)"/>
  <arg name="joint_states_ns" value="/m1n6s300/joint_states"/>
  <arg name="controller_manager" value="m1n6s300_ros_control"/>
  <arg name="gazebo_sim" value="$(arg gazebo_sim)"/>
</include> 
```

Adicional a esto se debe de realizar la instalación de las dependencias necesarias para la construcción de los paquetes

```bash
  rosdep install -y --from-paths . --ignore-src --rosdistro noetic
```
Las dependencias faltantes de versiones anteriores deben de instalarse munualmente.

Si al iniciar el rviz con moveit no se encuentra el interactor de movimiento, ejecutar el siguiente comando.

```bash
  sudo apt install ros-noetic-trac-ik-kinematics-plugin
```

Además de agregar el display necesario para los marcadores interactivos.

![Display Interactive Marker](images/display_interactive_marker.jpeg)

Otra configuración indispensable es agregar las configuraciones necesarias a los archivos xacro para poder spawnear de forma correcta las visualizaciones y configuraciones del brazo.

Refiriendonos primero al archivo [m1n6s300_standalone.xacro](https://github.com/Kinovarobotics/kinova-ros/blob/noetic-devel/kinova_description/urdf/m1n6s300_standalone.xacro) correspondiente al paquete [robot_description](https://github.com/Kinovarobotics/kinova-ros/blob/noetic-devel/kinova_description), es necesario agregar algunas configuraciones sobre la montura de la camara, disponible en la rama [ros1-legacy](https://github.com/realsenseai/realsense-ros/tree/ros1-legacy) del repositorio correspondiente de intel realsense d415 para ros1.

### Ejecución de rutina actual de movimiento

Para realizar la ejecución de la simulación debe tener las siguientes dependencias instaladas, en un entorno (con entornos de python es suficiente, aunque si se cree necesario se puede crear un entorno usando conda):

* numpy: versión 1.23.5
* opencv-python: versión 4.13.0.92
* ros_numpy: versión 0.0.5
* ultralytics: versión 8.4.23
* open3d: versión 0.19.0

Para lanzar los nodos realizar la ejecución de con los siguientes comandos:

Para lanzar el nodo con Gazebo

```bash
  roslaunch sim_kinova gazebo_kinova_sim.launch
```

a este comando se le puede agregar el argumento world para indicar que mundos cargar, por el momento los mundos disponibles son los siguientes:

```bash
world_table_aruco
world_table_bowl_with_apple
world_table_bowl
world_table_chessboard
```

puede usarse el comando el siguiente comando para lanzar cualquiera de los mundos, 

```bash
  roslaunch sim_kinova gazebo_kinova_sim.launch world:=world_table_bowl_with_apple
```

el mundo cargado por defecto es ***world_table_bowl_with_apple***.

Posteriormente, se debe de lanzar el nodo de rviz.

```bash
  roslaunch sim_kinova rviz_kinova_sim.launch
```