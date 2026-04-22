#!/usr/bin/env python3
import rospy
import ros_numpy
import numpy as np
import cv2
import rospkg as rp
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from ultralytics import YOLO, SAM

# ==============================================================
#   NUEVOS IMPORTS PARA NUBE DE PUNTOS Y KD-TREE
# ==============================================================
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
import sensor_msgs.point_cloud2 as pc2
import open3d as o3d
# ==============================================================

class KinovaVisionD415:
    def __init__(self):
        rospy.init_node('vision_d415')
        #Publicamos en el tópico 'object_centroid' la posición 3D del centroide del objeto detectado, en el marco de la cámara
        self.pub = rospy.Publisher('object_centroid', PointStamped, queue_size=10)
        #Tópicos fijos de la RealSense D415 (Requiere align_depth:=true)
        self.TOPIC_RGB = "/camera/color/image_raw"
        self.TOPIC_DEPTH = "/camera/aligned_depth_to_color/image_raw"
        self.TOPIC_INFO = "/camera/color/camera_info"
        # ==============================================================
        #   NUEVO TÓPICO DE NUBE DE PUNTOS (ros1-legacy realsense-ros)
        #   Requiere lanzar con: filters:=pointcloud
        # ==============================================================
        self.TOPIC_POINTS = "/camera/depth/color/points"
        # ==============================================================
        #Carga de modelos optimizados para CPU/Laptop
        rospy.loginfo("Cargando modelos de YOLOv8 y MobileSAM")
        #Crear la instancia de RosPack
        rospack = rp.RosPack()
        #Obtener la ruta del paquete
        package_path = rospack.get_path('sam_segmentation')
        #Cargar los modelos
        self.yolo = YOLO(package_path + '/weights/yolov8s.pt')
        self.sam = SAM(package_path + '/weights/sam2_t.pt')
        #Obtener Intrínsecos de la cámara
        rospy.loginfo("Sincronizando con RealSense D415...")
        try:
            #Esperamos a recibir el mensaje de CameraInfo para obtener los parámetros de la cámara (focales y punto principal)
            info = rospy.wait_for_message(self.TOPIC_INFO, CameraInfo, timeout=10)
            self.fx, self.fy = info.K[0], info.K[4]
            self.cx, self.cy = info.K[2], info.K[5]
            self.cam_frame = info.header.frame_id
        except rospy.ROSException:
            rospy.logerr("No se detectó la cámara. Revisa que el driver esté corriendo.")
            return
        
        # ==============================================================
        #   NUEVA VARIABLE PARA ALMACENAR LA NUBE DE PUNTOS
        # ==============================================================
        self.last_cloud = None
        # ==============================================================

        #Estado y Suscriptores
        self.last_depth = None # Variable para almacenar la última imagen de profundidad recibida, necesaria para sincronizar con el RGB
        rospy.Subscriber(self.TOPIC_DEPTH, Image, self.depth_cb) # Suscripción a la imagen de profundidad alineada con el color, sin cola para procesar cada frame de profundidad que llega
        rospy.Subscriber(self.TOPIC_RGB, Image, self.rgb_cb, queue_size=1, buff_size=2**24) # Suscripción a la imagen RGB con cola de tamaño 1 para no saturar la memoria si el procesamiento es lento
        #rospy.loginfo(">>> Sistema de Visión Listo.")

        # ==============================================================
        #   NUEVO SUBSCRIBER PARA LA NUBE DE PUNTOS
        # ==============================================================
        rospy.Subscriber(self.TOPIC_POINTS, PointCloud2, self.cloud_cb, queue_size=1)
        # ==============================================================

    def depth_cb(self, msg):
        # Almacena el mapa de profundidad alineado
        self.last_depth = ros_numpy.numpify(msg) #Convertimos el mensaje ROS a un array de NumPy para procesarlo con OpenCV y sincronizarlo con el RGB

    # ==============================================================
    #   NUEVO CALLBACK PARA RECIBIR Y ALMACENAR LA NUBE DE PUNTOS
    # ==============================================================
    def cloud_cb(self, msg):
        puntos = list(pc2.read_points(
            msg,
            field_names=("x", "y", "z"),
            skip_nans=True   # descarta puntos sin retorno (NaN)
        ))
        if len(puntos) > 0:
            self.last_cloud = np.array(puntos, dtype=np.float32)  # shape (N, 3)
    # ==============================================================

    # ==============================================================
    #   NUEVA FUNCIÓN: PROFUNDIDAD CONFIABLE VÍA KD-TREE + NUBE 3D
    #   Flujo:
    #     1. Proyecta píxel (u,v) a punto 3D semilla usando z_seed
    #     2. Construye KD-Tree con la nube completa
    #     3. Busca K=10 vecinos más cercanos a la semilla
    #     4. Promedia su Z → profundidad confiable y suavizada
    # ==============================================================
    def get_depth_from_cloud(self, u, v, z_seed, k=10):
        if self.last_cloud is None:
            return 0.0

        x_seed = (u - self.cx) * z_seed / self.fx
        y_seed = (v - self.cy) * z_seed / self.fy
        seed   = np.array([[x_seed, y_seed, z_seed]])

        nube = o3d.geometry.PointCloud()
        nube.points = o3d.utility.Vector3dVector(self.last_cloud)
        kd_tree = o3d.geometry.KDTreeFlann(nube)

        _, idx, _ = kd_tree.search_knn_vector_3d(seed[0], k)

        vecinos  = self.last_cloud[idx]           # shape (K, 3)
        z_result = float(np.mean(vecinos[:, 2]))  # promedio del eje Z
        return z_result
    # ==============================================================

    def rgb_cb(self, msg):
        if self.last_depth is None:
            return
        # Convertir imagen a formato OpenCV
        frame_rgb = ros_numpy.numpify(msg) # Convertimos el mensaje ROS a un array de NumPy para procesarlo con OpenCV
        frame_bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR) # Convertimos a BGR para que OpenCV lo procese correctamente

        # YOLOv8 detecta el bowl o comida
        """Clases COCO relevantes para nuestro caso"""
        # Incluimos: tazón, copa, tenedor, cuchillo, cuchara y todas las frutas/comidas
        clases_interes = [0,41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55]

        ###################
        #Definir tamaño de entrada pequeño
        input_size = 320 
        img_small = cv2.resize(frame_bgr, (input_size, input_size))

        #Calcular factores de escala para volver a la imagen real después
        h_orig, w_orig = frame_bgr.shape[:2]
        sw, sh = w_orig / input_size, h_orig / input_size

        # YOLO ahora procesa la imagen PEQUEÑA (img_small)
        results = self.yolo(img_small, verbose=False, conf=0.5, classes=clases_interes)[0]
        ##############

        #results = self.yolo(frame_bgr, verbose=False, conf=0.5, classes=clases_interes)[0]# Solo procesamos las detecciones de bowl y comida, con un umbral de confianza del 50%

        for box in results.boxes:
            class_id = int(box.cls[0])
            class_name = self.yolo.names[class_id] 

            b = box.xyxy[0].cpu().numpy()
            
            # MULTIPLICAR por la escala para volver al tamaño real
            x1, y1 = int(b[0] * sw), int(b[1] * sh)
            x2, y2 = int(b[2] * sw), int(b[3] * sh)
            
            # Ahora cx_box y cy_box estarán en el lugar correcto de la imagen grande
            cx_box, cy_box = (x1 + x2) / 2, (y1 + y2) / 2

            # Obtener el recuadro para guiar a SAM
            #x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)

            # Calcular el centro de la caja de YOLO para ayudar a SAM
            #center_point = [[(x1 + x2) / 2, (y1 + y2) / 2]]

            # Calcular el centro del cuadro de YOLO
            # cx_box = (x1 + x2) / 2
            # cy_box = (y1 + y2) / 2

            # Predicción con SAM 2
            sam_results = self.sam.predict(
                frame_bgr, 
                bboxes=[[x1, y1, x2, y2]], 
                points=[[cx_box, cy_box]], # Punto central para guiar la segmentación
                labels=[1],                # 1 significa que el punto es "positivo" (es el objeto)
                verbose=False,
                imgsz=320
            )[0]

            """
            #Con bboxes 
            #MobileSAM segmenta con precisión quirúrgica
            sam_results = self.sam.predict(frame_bgr, bboxes=[[x1, y1, x2, y2]], verbose=False)[0] # Pedimos a SAM que segmente dentro del recuadro detectado por YOLO, sin mostrar logs adicionales
            """
            if sam_results.masks is not None: # Si SAM nos devuelve máscaras, procesamos la primera (en este caso, debería ser solo una máscara por detección)
                #La mascara que nos da SAM la usamos para calcular el centroide del objeto segmentado, ya que es más preciso que el cuadro de YOLO, especialmente para objetos irregulares como comida o bowls
                #esta está compuesta por píxeles con valor 1 donde está el objeto y 0 en el fondo, lo convertimos a uint8 para procesarla con OpenCV y calcular el centroide correctamente
                mask = sam_results.masks.data[0].cpu().numpy().astype(np.uint8) 
                # Calcular Centroide
                M = cv2.moments(mask) 
                if M["m00"] == 0: continue
                u = int(M["m10"] / M["m00"]) # Coordenada X del centroide en píxeles
                v = int(M["m01"] / M["m00"]) # Coordenada Y del centroide en píxeles

                # ==============================================================
                #   MODIFICADO: get_filtered_depth ahora es solo SEMILLA para KD-Tree
                #   La profundidad final z_m viene de get_depth_from_cloud
                # ==============================================================
                # z_m = self.get_filtered_depth(u, v)  # <-- antes era así (directo)
                z_seed = self.get_filtered_depth(u, v)  # ahora solo semilla
                # ==============================================================

                """AJUSTAR"""
                if 0.2 < z_seed < 1.2: # Filtro de rango de operación 
                    #Al poner 0.2 < z_m, nos asegura de que el objeto esté en el rango donde la cámara realmente puede "ver" con precisión

                    # ==============================================================
                    #   NUEVO: Z FINAL VIENE DEL KD-TREE SOBRE LA NUBE DE PUNTOS
                    #   Si la nube aún no llegó, usa z_seed como fallback
                    # ==============================================================
                    if self.last_cloud is not None:
                        z_m = self.get_depth_from_cloud(u, v, z_seed, k=10)
                    else:
                        z_m = z_seed  # fallback si la nube aún no llegó
                    # ==============================================================

                    # Proyectar a coordenadas 3D de la cámara
                    x_c = (u - self.cx) * z_m / self.fx #Colocamos la profundidad en metros para que la proyección a 3D sea correcta, ya que los intrínsecos están en píxeles y metros
                    y_c = (v - self.cy) * z_m / self.fy
                    
                    # --- LLAMADA A LA FUNCIÓN DE PUBLICACIÓN ---
                    self.publish_msg(x_c, y_c, z_m) 
                    # ================== Visualización ==================
                    # Dibujar contorno de la máscara de SAM
                    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                    cv2.drawContours(frame_bgr, contours, -1, (0, 255, 0), 2)
                    # Crear el texto de la etiqueta
                    label = f"{class_name.upper()} | Z: {z_m:.2f}m"
                    # Dibujar un pequeño rectángulo de fondo para que el texto sea legible
                    (w, h), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 1)
                    cv2.rectangle(frame_bgr, (x1, y1 - 25), (x1 + w, y1), (0, 255, 0), -1)
                    # Poner el texto (Clase y Distancia)
                    cv2.putText(frame_bgr, label, (x1, y1 - 7), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)
                    # Punto rojo en el centroide exacto
                    cv2.circle(frame_bgr, (u, v), 5, (0, 0, 255), -1)
        cv2.imshow("D415 Vision - YOLO+SAM", frame_bgr)
        cv2.waitKey(1)

    def get_filtered_depth(self, u, v):
        """ Filtra ceros y ruidos de la D415 usando una ventana de 5x5 """
        try:
            roi = self.last_depth[v-2:v+3, u-2:u+3] # Obtenemos una región de interés de 5x5 píxeles alrededor del centroide para filtrar ruidos y ceros
            valid_depths = roi[roi > 0] # Filtramos los valores de profundidad válidos (mayores que 0, ya que 0 suele indicar falta de datos o ruido)
            if len(valid_depths) > 0: 
                return np.median(valid_depths) * 0.001 # mm a metros (regresa la mediana para mayor estabilidad)
            return 0.0
        except IndexError:
            return 0.0
          
    """Función para publicar el mensaje con el centroide 3D del objeto detectado en el tópico 'object_centroid'"""
    def publish_msg(self, x, y, z):
        target_msg = PointStamped() 
        target_msg.header.stamp = rospy.Time.now() 
        target_msg.header.frame_id = self.cam_frame   
        target_msg.point.x = x
        target_msg.point.y = y 
        target_msg.point.z = z 
        self.pub.publish(target_msg)
        rospy.loginfo_throttle(1, f"Publicando: X={x:.3f}, Y={y:.3f}, Z={z:.3f}") # Log para debug

if __name__ == '__main__':
    try:
        KinovaVisionD415()
        rospy.spin()
    except rospy.ROSInterruptException:
        cv2.destroyAllWindows()