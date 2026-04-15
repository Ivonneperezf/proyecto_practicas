#!/usr/bin/env python3
import rospy
import ros_numpy
import numpy as np
import cv2
import rospkg as rp
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from geometry_msgs.msg import PointStamped
from ultralytics import YOLO, SAM
import sensor_msgs.point_cloud2 as pc2
import open3d as o3d

class KinovaVisionD415:
    def __init__(self):
        rospy.init_node('vision_simulation')
        self.pub = rospy.Publisher('object_centroid', PointStamped, queue_size=10)

        self.TOPIC_RGB    = "/d415/color/image_raw"
        self.TOPIC_DEPTH  = "/d415/depth/image_raw"
        self.TOPIC_INFO   = "/d415/color/camera_info"
        # ============================================================
        # [NUBE DE PUNTOS] Tópico de la nube de puntos del plugin depth
        # publicado por libgazebo_ros_openni_kinect.so en el XACRO
        # ============================================================
        self.TOPIC_POINTS = "/d415/depth/points"

        rospy.loginfo("Cargando modelos de YOLOv8 y MobileSAM")
        rospack      = rp.RosPack()
        package_path = rospack.get_path('sam_segmentation')
        self.yolo    = YOLO(package_path + '/weights/yolov8s.pt')
        self.sam     = SAM(package_path  + '/weights/sam2_t.pt')

        rospy.loginfo("Sincronizando con cámara Gazebo D415...")
        try:
            info = rospy.wait_for_message(self.TOPIC_INFO, CameraInfo, timeout=10)
            self.fx, self.fy = info.K[0], info.K[4]
            self.cx, self.cy = info.K[2], info.K[5]
            self.cam_frame   = info.header.frame_id
        except rospy.ROSException:
            rospy.logerr("No se detectó la cámara. Revisa que Gazebo esté corriendo.")
            return

        self.last_depth  = None
        # ============================================================
        # [NUBE DE PUNTOS] Variable para almacenar la última nube
        # recibida como array NumPy (N, 3) con columnas X, Y, Z
        # ============================================================
        self.last_cloud  = None

        rospy.Subscriber(self.TOPIC_DEPTH,  Image,        self.depth_cb)
        # ============================================================
        # [NUBE DE PUNTOS] Suscripción a la nube de puntos.
        # Sin cola (queue_size=1) para siempre tener la nube más reciente
        # ============================================================
        rospy.Subscriber(self.TOPIC_POINTS, PointCloud2,  self.cloud_cb, queue_size=1)
        rospy.Subscriber(self.TOPIC_RGB,    Image,        self.rgb_cb, queue_size=1, buff_size=2**24)

    def depth_cb(self, msg):
        # Sigue activo como puente para proyectar x_c e y_c
        self.last_depth = ros_numpy.numpify(msg)  # float32, metros (Gazebo)

    # ================================================================
    # [NUBE DE PUNTOS] Callback que recibe el PointCloud2 y lo convierte
    # a un array NumPy de shape (N, 3) con los puntos X, Y, Z en metros.
    # Se guarda en self.last_cloud para consultarlo en rgb_cb.
    # ================================================================
    def cloud_cb(self, msg):
        puntos = list(pc2.read_points(
            msg,
            field_names=("x", "y", "z"),
            skip_nans=True   # descarta puntos sin retorno (NaN)
        ))
        if len(puntos) > 0:
            self.last_cloud = np.array(puntos, dtype=np.float32)  # shape (N, 3)

    # ================================================================
    # [NUBE DE PUNTOS] Obtiene la profundidad Z confiable del centroide
    # usando KD-Tree + K vecinos más cercanos sobre la nube 3D.
    #
    # Flujo:
    #   1. Proyectamos el píxel (u, v) a un punto 3D aproximado
    #      usando z_m de la imagen de profundidad (solo como semilla)
    #   2. Construimos un KD-Tree con la nube de puntos completa
    #   3. Buscamos los K puntos más cercanos a esa semilla 3D
    #   4. Promediamos su Z → profundidad confiable y suavizada
    #
    # Esto es más robusto que la ventana 5x5 porque:
    #   - Opera en espacio 3D real, no en píxeles
    #   - Los K vecinos son puntos reales del objeto, no píxeles adyacentes
    #   - El promedio elimina outliers mejor que la mediana de 25 píxeles
    # ================================================================
    def get_depth_from_cloud(self, u, v, z_seed, k=10):
        if self.last_cloud is None:
            return 0.0

        # Paso 1: punto semilla 3D desde el píxel (u, v) con z_seed
        x_seed = (u - self.cx) * z_seed / self.fx
        y_seed = (v - self.cy) * z_seed / self.fy
        seed   = np.array([[x_seed, y_seed, z_seed]])

        # Paso 2: construir KD-Tree con la nube completa
        nube = o3d.geometry.PointCloud()
        nube.points = o3d.utility.Vector3dVector(self.last_cloud)
        kd_tree = o3d.geometry.KDTreeFlann(nube)

        # Paso 3: buscar los K vecinos más cercanos al punto semilla
        # kd_tree.search_knn_vector_3d devuelve (num_encontrados, indices, distancias)
        _, idx, _ = kd_tree.search_knn_vector_3d(seed[0], k)

        # Paso 4: extraer los K puntos y promediar su Z
        vecinos  = self.last_cloud[idx]        # shape (K, 3)
        z_result = float(np.mean(vecinos[:, 2]))  # promedio del eje Z
        return z_result

    def rgb_cb(self, msg):
        if self.last_depth is None:
            return

        frame_rgb = ros_numpy.numpify(msg)
        frame_bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)

        clases_interes = [0, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55]

        input_size     = 320
        img_small      = cv2.resize(frame_bgr, (input_size, input_size))
        h_orig, w_orig = frame_bgr.shape[:2]
        sw, sh         = w_orig / input_size, h_orig / input_size

        results = self.yolo(img_small, verbose=False, conf=0.5, classes=clases_interes)[0]

        for box in results.boxes:
            class_id   = int(box.cls[0])
            class_name = self.yolo.names[class_id]

            b      = box.xyxy[0].cpu().numpy()
            x1, y1 = int(b[0] * sw), int(b[1] * sh)
            x2, y2 = int(b[2] * sw), int(b[3] * sh)
            cx_box = (x1 + x2) / 2
            cy_box = (y1 + y2) / 2

            sam_results = self.sam.predict(
                frame_bgr,
                bboxes=[[x1, y1, x2, y2]],
                points=[[cx_box, cy_box]],
                labels=[1],
                verbose=False,
                imgsz=320
            )[0]

            if sam_results.masks is not None:
                mask = sam_results.masks.data[0].cpu().numpy().astype(np.uint8)
                M    = cv2.moments(mask)
                if M["m00"] == 0:
                    continue

                u = int(M["m10"] / M["m00"])
                v = int(M["m01"] / M["m00"])

                # Paso previo: z_seed desde imagen de profundidad (semilla para KD-Tree)
                z_seed = self.get_filtered_depth(u, v)

                if 0.2 < z_seed < 1.2:

                    # ================================================
                    # [NUBE DE PUNTOS] Obtenemos Z confiable con KD-Tree
                    # Si la nube no está lista aún, usamos z_seed directo
                    # ================================================
                    if self.last_cloud is not None:
                        z_m = self.get_depth_from_cloud(u, v, z_seed, k=10)
                    else:
                        z_m = z_seed  # fallback mientras llega la nube

                    # X e Y proyectados con z_m confiable de la nube
                    x_c = (u - self.cx) * z_m / self.fx
                    y_c = (v - self.cy) * z_m / self.fy

                    self.publish_msg(x_c, y_c, z_m)

                    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                    cv2.drawContours(frame_bgr, contours, -1, (0, 255, 0), 2)
                    label     = f"{class_name.upper()} | X:{x_c:.2f} Y:{y_c:.2f} Z:{z_m:.2f}"
                    (w, h), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 1)
                    cv2.rectangle(frame_bgr, (x1, y1 - 25), (x1 + w, y1), (0, 255, 0), -1)
                    cv2.putText(frame_bgr, label, (x1, y1 - 7),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)
                    cv2.circle(frame_bgr, (u, v), 5, (0, 0, 255), -1)

        cv2.imshow("D415 Gazebo - YOLO+SAM", frame_bgr)
        cv2.waitKey(1)

    def get_filtered_depth(self, u, v):
        """Ventana 5x5 — ahora solo actúa como semilla para el KD-Tree."""
        try:
            roi          = self.last_depth[v-2:v+3, u-2:u+3]
            valid_depths = roi[np.isfinite(roi) & (roi > 0)]
            if len(valid_depths) > 0:
                return float(np.median(valid_depths))
            return 0.0
        except IndexError:
            return 0.0

    def publish_msg(self, x, y, z):
        """Publica X, Y, Z del centroide. Z ahora viene de la nube de puntos."""
        target_msg             = PointStamped()
        target_msg.header.stamp    = rospy.Time.now()
        target_msg.header.frame_id = self.cam_frame
        target_msg.point.x = x
        target_msg.point.y = y
        target_msg.point.z = z
        self.pub.publish(target_msg)
        rospy.loginfo_throttle(1, f"Publicando: X={x:.3f}  Y={y:.3f}  Z={z:.3f}")

if __name__ == '__main__':
    try:
        KinovaVisionD415()
        rospy.spin()
    except rospy.ROSInterruptException:
        cv2.destroyAllWindows()





# import rospy
# import ros_numpy
# import numpy as np
# import cv2
# import rospkg as rp
# from sensor_msgs.msg import Image, CameraInfo
# from geometry_msgs.msg import PointStamped
# from ultralytics import YOLO, SAM

# class KinovaVisionD415:
#     def __init__(self):
#         rospy.init_node('vision_simulation')
#         self.pub = rospy.Publisher('object_centroid', PointStamped, queue_size=10)

#         self.TOPIC_RGB   = "/d415/color/image_raw"
#         self.TOPIC_DEPTH = "/d415/depth/image_raw"
#         self.TOPIC_INFO  = "/d415/color/camera_info"

#         rospy.loginfo("Cargando modelos de YOLOv8 y MobileSAM")
#         rospack      = rp.RosPack()
#         package_path = rospack.get_path('sam_segmentation')
#         self.yolo    = YOLO(package_path + '/weights/yolov8s.pt')
#         self.sam     = SAM(package_path  + '/weights/sam2_t.pt')

#         rospy.loginfo("Sincronizando con cámara Gazebo D415...")
#         try:
#             info = rospy.wait_for_message(self.TOPIC_INFO, CameraInfo, timeout=10)
#             self.fx, self.fy = info.K[0], info.K[4]
#             self.cx, self.cy = info.K[2], info.K[5]
#             self.cam_frame   = info.header.frame_id
#         except rospy.ROSException:
#             rospy.logerr("No se detectó la cámara. Revisa que Gazebo esté corriendo.")
#             return

#         self.last_depth = None
#         rospy.Subscriber(self.TOPIC_DEPTH, Image, self.depth_cb)
#         rospy.Subscriber(self.TOPIC_RGB,   Image, self.rgb_cb, queue_size=1, buff_size=2**24)

#     def depth_cb(self, msg):
#         self.last_depth = ros_numpy.numpify(msg)  # float32, metros (Gazebo)

#     def rgb_cb(self, msg):
#         if self.last_depth is None:
#             return

#         frame_rgb = ros_numpy.numpify(msg)
#         frame_bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)

#         clases_interes = [0, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55]

#         input_size     = 320
#         img_small      = cv2.resize(frame_bgr, (input_size, input_size))
#         h_orig, w_orig = frame_bgr.shape[:2]
#         sw, sh         = w_orig / input_size, h_orig / input_size

#         results = self.yolo(img_small, verbose=False, conf=0.5, classes=clases_interes)[0]

#         for box in results.boxes:
#             class_id   = int(box.cls[0])
#             class_name = self.yolo.names[class_id]

#             b      = box.xyxy[0].cpu().numpy()
#             x1, y1 = int(b[0] * sw), int(b[1] * sh)
#             x2, y2 = int(b[2] * sw), int(b[3] * sh)
#             cx_box = (x1 + x2) / 2
#             cy_box = (y1 + y2) / 2

#             sam_results = self.sam.predict(
#                 frame_bgr,
#                 bboxes=[[x1, y1, x2, y2]],
#                 points=[[cx_box, cy_box]],
#                 labels=[1],
#                 verbose=False,
#                 imgsz=320
#             )[0]

#             if sam_results.masks is not None:
#                 mask = sam_results.masks.data[0].cpu().numpy().astype(np.uint8)
#                 M    = cv2.moments(mask)
#                 if M["m00"] == 0:
#                     continue

#                 u = int(M["m10"] / M["m00"])
#                 v = int(M["m01"] / M["m00"])

#                 # ====================================================
#                 # [TODO - NUBE DE PUNTOS] get_filtered_depth() será
#                 # reemplazada por /d415/depth/points en el futuro.
#                 # Por ahora se usa como puente para obtener z_m y poder
#                 # proyectar x_c e y_c correctamente.
#                 # ====================================================
#                 z_m = self.get_filtered_depth(u, v)

#                 if 0.2 < z_m < 1.2:

#                     # X e Y proyectados desde píxel a metros
#                     x_c = (u - self.cx) * z_m / self.fx
#                     y_c = (v - self.cy) * z_m / self.fy

#                     # ================================================
#                     # Z se congela en 0.0 hasta integrar nube de puntos
#                     # El transformer y el mover solo actuarán sobre X e Y
#                     # ================================================
#                     self.publish_msg(x_c, y_c, z_frozen=0.0)

#                     # Visualización
#                     contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
#                     cv2.drawContours(frame_bgr, contours, -1, (0, 255, 0), 2)
#                     label     = f"{class_name.upper()} | X:{x_c:.2f} Y:{y_c:.2f}"
#                     (w, h), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 1)
#                     cv2.rectangle(frame_bgr, (x1, y1 - 25), (x1 + w, y1), (0, 255, 0), -1)
#                     cv2.putText(frame_bgr, label, (x1, y1 - 7),
#                                 cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)
#                     cv2.circle(frame_bgr, (u, v), 5, (0, 0, 255), -1)

#         cv2.imshow("D415 Gazebo - YOLO+SAM", frame_bgr)
#         cv2.waitKey(1)

#     # ================================================================
#     # [TODO - NUBE DE PUNTOS]
#     # Reemplazar con extracción desde /d415/depth/points
#     # ================================================================
#     def get_filtered_depth(self, u, v):
#         """[TEMPORAL] Solo para proyectar x_c e y_c, Z no se publica."""
#         try:
#             roi          = self.last_depth[v-2:v+3, u-2:u+3]
#             valid_depths = roi[np.isfinite(roi) & (roi > 0)]
#             if len(valid_depths) > 0:
#                 return float(np.median(valid_depths))
#             return 0.0
#         except IndexError:
#             return 0.0

#     def publish_msg(self, x, y, z_frozen):
#         """Publica X e Y del centroide. Z congelado en 0.0 hasta nube de puntos."""
#         target_msg             = PointStamped()
#         target_msg.header.stamp    = rospy.Time.now()
#         target_msg.header.frame_id = self.cam_frame
#         target_msg.point.x = x
#         target_msg.point.y = y
#         target_msg.point.z = z_frozen  # [TODO] reemplazar con Z de nube de puntos
#         self.pub.publish(target_msg)
#         rospy.loginfo_throttle(1, f"Publicando: X={x:.3f}  Y={y:.3f}  Z=0.0 [congelado]")

# if __name__ == '__main__':
#     try:
#         KinovaVisionD415()
#         rospy.spin()
#     except rospy.ROSInterruptException:
#         cv2.destroyAllWindows()





# import rospy
# import ros_numpy
# import numpy as np
# import cv2
# import rospkg as rp
# from sensor_msgs.msg import Image, CameraInfo
# from geometry_msgs.msg import PointStamped
# from ultralytics import YOLO, SAM

# class KinovaVisionD415:
#     def __init__(self):
#         rospy.init_node('vision_simulation')
#         #Publicamos en el tópico 'object_centroid' la posición 3D del centroide del objeto detectado, en el marco de la cámara
#         self.pub = rospy.Publisher('object_centroid', PointStamped, queue_size=10)

#         # ============================================================
#         # [GAZEBO] Tópicos del plugin libgazebo_ros_camera.so
#         # En físico usábamos /camera/..., en Gazebo el cameraName
#         # es "d415/color", por lo que los tópicos cambian así:
#         # ============================================================
#         self.TOPIC_RGB   = "/d415/color/image_raw"       # CAMBIADO: antes /camera/color/image_raw
#         self.TOPIC_DEPTH = "/d415/depth/image_raw"        # CAMBIADO: antes /camera/aligned_depth_to_color/image_raw
#         self.TOPIC_INFO  = "/d415/color/camera_info"      # CAMBIADO: antes /camera/color/camera_info

#         # ============================================================
#         # [GAZEBO - SIN ALIGN] Gazebo NO tiene align_depth:=true.
#         # Color y Depth comparten el mismo FOV y resolución en el XACRO
#         # (ambos 640x480, fov=1.2043), por lo que tratamos la profundidad
#         # como si estuviera alineada al color. No se requiere ningún
#         # nodo extra de alineación.
#         # ============================================================

#         #Carga de modelos optimizados para CPU/Laptop
#         rospy.loginfo("Cargando modelos de YOLOv8 y MobileSAM")
#         rospack = rp.RosPack()
#         package_path = rospack.get_path('sam_segmentation')
#         self.yolo = YOLO(package_path + '/weights/yolov8s.pt')
#         self.sam  = SAM(package_path  + '/weights/sam2_t.pt')

#         #Obtener Intrínsecos de la cámara
#         rospy.loginfo("Sincronizando con cámara Gazebo D415...")
#         try:
#             info = rospy.wait_for_message(self.TOPIC_INFO, CameraInfo, timeout=10)
#             self.fx, self.fy = info.K[0], info.K[4]
#             self.cx, self.cy = info.K[2], info.K[5]
#             self.cam_frame   = info.header.frame_id
#         except rospy.ROSException:
#             rospy.logerr("No se detectó la cámara. Revisa que Gazebo esté corriendo y el plugin activo.")
#             return

#         #Estado y Suscriptores
#         self.last_depth = None
#         rospy.Subscriber(self.TOPIC_DEPTH, Image, self.depth_cb)
#         rospy.Subscriber(self.TOPIC_RGB,   Image, self.rgb_cb, queue_size=1, buff_size=2**24)

#     def depth_cb(self, msg):
#         # ============================================================
#         # [GAZEBO] ros_numpy devuelve float32 en metros directamente.
#         # En físico llegaba uint16 en mm. Guardamos igual, solo cambia
#         # la conversión en get_filtered_depth (ver abajo).
#         # ============================================================
#         self.last_depth = ros_numpy.numpify(msg)  # float32, metros (Gazebo)

#     def rgb_cb(self, msg):
#         if self.last_depth is None:
#             return
#         frame_rgb = ros_numpy.numpify(msg)
#         frame_bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)

#         # Clases COCO relevantes (sin cambios)
#         clases_interes = [0, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55]

#         input_size = 320
#         img_small  = cv2.resize(frame_bgr, (input_size, input_size))

#         h_orig, w_orig = frame_bgr.shape[:2]
#         sw, sh = w_orig / input_size, h_orig / input_size

#         results = self.yolo(img_small, verbose=False, conf=0.5, classes=clases_interes)[0]

#         for box in results.boxes:
#             class_id   = int(box.cls[0])
#             class_name = self.yolo.names[class_id]

#             b  = box.xyxy[0].cpu().numpy()
#             x1, y1 = int(b[0] * sw), int(b[1] * sh)
#             x2, y2 = int(b[2] * sw), int(b[3] * sh)

#             cx_box = (x1 + x2) / 2
#             cy_box = (y1 + y2) / 2

#             sam_results = self.sam.predict(
#                 frame_bgr,
#                 bboxes=[[x1, y1, x2, y2]],
#                 points=[[cx_box, cy_box]],
#                 labels=[1],
#                 verbose=False,
#                 imgsz=320
#             )[0]

#             if sam_results.masks is not None:
#                 mask = sam_results.masks.data[0].cpu().numpy().astype(np.uint8)
#                 M = cv2.moments(mask)
#                 if M["m00"] == 0:
#                     continue
#                 u = int(M["m10"] / M["m00"])
#                 v = int(M["m01"] / M["m00"])

#                 z_m = self.get_filtered_depth(u, v)

#                 # ============================================================
#                 # [GAZEBO] El rango 0.2–1.2 m sigue siendo válido.
#                 # El plugin en el XACRO tiene near=0.1 y far=10.0,
#                 # así que la cámara simulada cubre de sobra este rango.
#                 # ============================================================
#                 if 0.2 < z_m < 1.2:
#                     x_c = (u - self.cx) * z_m / self.fx
#                     y_c = (v - self.cy) * z_m / self.fy

#                     self.publish_msg(x_c, y_c, z_m)

#                     contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
#                     cv2.drawContours(frame_bgr, contours, -1, (0, 255, 0), 2)
#                     label = f"{class_name.upper()} | Z: {z_m:.2f}m"
#                     (w, h), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 1)
#                     cv2.rectangle(frame_bgr, (x1, y1 - 25), (x1 + w, y1), (0, 255, 0), -1)
#                     cv2.putText(frame_bgr, label, (x1, y1 - 7),
#                                 cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)
#                     cv2.circle(frame_bgr, (u, v), 5, (0, 0, 255), -1)

#         cv2.imshow("D415 Gazebo - YOLO+SAM", frame_bgr)  # CAMBIADO: etiqueta ventana indica Gazebo
#         cv2.waitKey(1)

#     def get_filtered_depth(self, u, v):
#         """Filtra NaN/ceros del mapa de profundidad usando ventana 5x5"""
#         try:
#             roi = self.last_depth[v-2:v+3, u-2:u+3]

#             # ============================================================
#             # [GAZEBO] La profundidad llega en float32 metros.
#             # Filtramos NaN (ausencia de dato en Gazebo) además de ceros.
#             # En físico solo filtrábamos ceros (uint16 mm).
#             # ============================================================
#             valid_depths = roi[np.isfinite(roi) & (roi > 0)]  # CAMBIADO: añadido np.isfinite para NaN de Gazebo

#             if len(valid_depths) > 0:
#                 return float(np.median(valid_depths))  # CAMBIADO: sin * 0.001, Gazebo ya entrega metros
#             return 0.0
#         except IndexError:
#             return 0.0

#     def publish_msg(self, x, y, z):
#         """Publica el centroide 3D del objeto en el tópico object_centroid"""
#         target_msg = PointStamped()
#         target_msg.header.stamp    = rospy.Time.now()
#         target_msg.header.frame_id = self.cam_frame
#         target_msg.point.x = x
#         target_msg.point.y = y
#         target_msg.point.z = z
#         self.pub.publish(target_msg)
#         rospy.loginfo_throttle(1, f"Publicando: X={x:.3f}, Y={y:.3f}, Z={z:.3f}")

# if __name__ == '__main__':
#     try:
#         KinovaVisionD415()
#         rospy.spin()
#     except rospy.ROSInterruptException:
#         cv2.destroyAllWindows()