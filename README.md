# Proyecto de practicas profesionales Primavera 2026

Estos son paquetes creados para ejecutar movimiento y modificar cosas, sin embargo, existen algunas modificaciones que deben de hacerse en el repositorio clonado propio del brazo, proveniente del repositorio oficial de [kinova-ros](https://github.com/kinovarobotics/kinova-ros).

Algunas de las modificaciones más importantes son las modificaciones realizadas para habilitar correctamente los controladores adecuados para [Moveit!](https://moveit.github.io/moveit_tutorials/), ya que existieron problemas puntuales sobre su instalación.

Para navegar fácilmente en el repositorio es conveniente, después de hacer source en el workspace, ejecutar el siguiente comando:

```bash
  roscd m1n6s300_moveit_config/launch
```

En el archivo ***m1n6s300_gazebo_demo.launch*** debemos agregar algunas líneas de código, ya que los controladores cargados para la simulación son los controladore usados por el brazo físico, a diferencia de los controladores necesarios para gazebo

```XML
  roscd m1n6s300_moveit_config/launch
```