#!/usr/bin/env python3
import rospy
import ros_numpy # Para convertir mensajes ROS a NumPy, necesario para procesar imágenes con OpenCV
import numpy as np
import cv2
from sensor_msgs.msg import Image, CameraInfo # Para suscribirnos a los frames de la cámara y obtener los parámetros de la lente
from geometry_msgs.msg import PointStamped # Para publicar las coordenadas del alimento en el sistema de referencia del robot
import tf2_ros # Para manejar transformaciones entre el sistema de la cámara y el sistema del robot, aplicando la calibración con ROS TF
#import tf2_geometry_msgs
from ultralytics.models.sam import SAM3SemanticPredictor
import moveit_commander # Para controlar el movimiento del robot usando MoveIt
import sys 

class Sam3BowlNode:
    def __init__(self):
        rospy.init_node('sam3_bowl_node')

        # Configuración de Cámara e Intrínsecos
        # Suscribirse a CameraInfo para obtener los parámetros de la lente automáticamente
        self.cam_info = rospy.wait_for_message("/camera/color/camera_info", CameraInfo)
        self.fx = self.cam_info.K[0] #Focal X
        self.fy = self.cam_info.K[4] #Focal Y
        self.cx = self.cam_info.K[2] #Principal Point X
        self.cy = self.cam_info.K[5] #Principal Point Y
        rospy.loginfo(f"Parámetros de la cámara: fx={self.fx}, fy={self.fy}, cx={self.cx}, cy={self.cy}")
        #Buffer de Transformadas (TF) para convertir coordenadas de la cámara al sistema del robot usando la calibración
        self.tf_buffer = tf2_ros.Buffer() # Buffer para almacenar las transformaciones
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer) # Listener para recibir las transformaciones publicadas
        #SAM 3
        overrides = dict(
            conf=0.25, #Bajamos el umbral para no perdernos nada, aunque aumente falsos positivos
            task="segment", #Siempre segmentación, no clasificación
            mode="predict", #Modo predictivo, no entrenamiento
            model="../weights/sam3.pt",
            half=False #En CPU Ryzen U-Series, False es más estable que True (aunque más lento)
        )
        self.predictor = SAM3SemanticPredictor(overrides=overrides) # Cargamos el modelo una sola vez al iniciar el nodo
        #Suscriptores
        self.depth_image = None # Variable para almacenar la última imagen de profundidad recibida
        self.sub_depth = rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, self.depth_callback) # Suscripción a la imagen de profundidad alineada con el color
        self.sub_rgb = rospy.Subscriber("/camera/color/image_raw", Image, self.callback, queue_size=1) # Suscripción con cola de tamaño 1 para no saturar la memoria
        rospy.loginfo("SAM 3 (Segment Anything with Concepts) listo")

    def depth_callback(self, msg):
        # La profundidad de RealSense suele venir en milímetros (uint16)
        self.depth_image = ros_numpy.numpify(msg)

    def callback(self, msg):
        if self.depth_image is None: # Si aún no hemos recibido una imagen de profundidad, no podemos procesar el RGB, así que esperamos
            return
        frame = ros_numpy.numpify(msg) # Convertimos el mensaje ROS a un array de NumPy para procesarlo con OpenCV
        frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR) # Convertimos a BGR para que OpenCV lo procese correctamente
        self.predictor.set_image(frame_bgr) # Configuramos la imagen en el predictor
        results = self.predictor(text=["bowl", "food"]) # Pedimos segmentación por concepto, no por caja delimitadora
        if len(results) > 0 and results[0].masks is not None: # Si SAM 3 nos devuelve máscaras, las procesamos
            for i, mask_data in enumerate(results[0].masks.data): # Iteramos sobre cada máscara devuelta por SAM 3 (una para "bowl" y otra para "food")
                mask = mask_data.cpu().numpy().astype(np.uint8) # Máscara binaria (0 o 1)
                """Calcular el Centroide (u, v) de la máscara usando momentos de OpenCV"""
                M = cv2.moments(mask) # Calculamos los momentos de la máscara para obtener su centroide
                if M["m00"] == 0: continue # Si el área de la máscara es cero, no podemos calcular un centroide válido (division entre 0), así que saltamos esta máscara
                u = int(M["m10"] / M["m00"]) # Coordenada X del centroide en píxeles
                v = int(M["m01"] / M["m00"]) # Coordenada Y del centroide en píxeles

                """Obtener la profundidad (Z) en ese punto"""
                # Usamos un promedio de 5x5 píxeles alrededor del centro para mayor estabilidad
                roi_depth = self.depth_image[v-2:v+3, u-2:u+3]
                z_mm = np.nanmedian(roi_depth) # Mediana para ignorar valores erróneos (0)
                z_m = z_mm * 0.001 # Convertir a metros

                if z_m > 0:
                    #Proyección de Píxel a 3D (Coordenadas de la cámara)
                    x_c = (u - self.cx) * z_m / self.fx # Coordenada X en metros en el sistema de la cámara
                    y_c = (v - self.cy) * z_m / self.fy# Coordenada Y en metros en el sistema de la cámara
                    #z_m ya es la coordenada Z en metros en el sistema de la cámara
                    
                    #Transformar al sistema del Kinova
                    self.get_robot_coords(x_c, y_c, z_m) 

                    # Visualización del centro
                    cv2.circle(frame_bgr, (u, v), 5, (0, 0, 255), -1) # Dibujamos un círculo rojo en el centroide detectado
            
            cv2.imshow("SAM 3: Centroide detectado", frame_bgr) 
            cv2.waitKey(1) # Necesario para que OpenCV actualice la ventana, aunque no esperamos una tecla específica

    """ Función para transformar las coordenadas del alimento desde el sistema de la cámara al sistema del robot usando la calibración con ROS TF """
    def get_robot_coords(self, x, y, z): 
        point_cam = PointStamped() # Creamos un mensaje de tipo PointStamped para representar el punto en el sistema de la cámara
        point_cam.header.frame_id = "camera_color_optical_frame" # Frame de la RealSense
        point_cam.header.stamp = rospy.Time(0) # Usamos el tiempo 0 para obtener la transformación más reciente disponible
        point_cam.point.x = x # Coordenada X en metros en el sistema de la cámara
        point_cam.point.y = y # Coordenada Y en metros en el sistema de la cámara
        point_cam.point.z = z # Coordenada Z en metros en el sistema de la cámara

        try:
            # Aquí se aplicará la calibración automáticamente
            # "root" o "mico_base_link" suele ser el nombre de la base del Kinova CAMBIAR SI ES NECESARIO
            """Transformamos el punto al sistema de referencia del robot usando la TF de calibración, 
            con un timeout para evitar bloqueos por falta de transformaciones disponibles evitando errores 
            críticos si la TF no está publicada correctamente"""
            point_robot = self.tf_buffer.transform(point_cam, "root", timeout=rospy.Duration(0.1)) 
            rospy.loginfo(f"ALIMENTO ENCONTRADO -> Robot Base: X={point_robot.point.x:.3f}m, Y={point_robot.point.y:.3f}m, Z={point_robot.point.z:.3f}m")
            
            # Aquí llamaremos a la función de movimiento (ej: MoveIt o Kinova API)
            # self.move_kinova(point_robot.point)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn_throttle(5, "Esperando a que el TF de calibración esté disponible...") # Mensaje de advertencia cada 5 segundos si la TF no está disponible, para no saturar los logs

if __name__ == '__main__':
    node = Sam3BowlNode() 
    rospy.spin() # Mantenemos el nodo en ejecución hasta que se cierre manualmente o ocurra un error crítico