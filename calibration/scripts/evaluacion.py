#!/usr/bin/env python3

import rospy
import tf2_ros
import tf_conversions
import numpy as np
import cv2
import yaml
from geometry_msgs.msg import PointStamped, TransformStamped
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class ArucoCenterDetector:

    def __init__(self):

        rospy.init_node("aruco_center_detector")

        # Frames del robot
        self.base_frame = "m1n6s300_link_base"
        self.ee_frame   = "m1n6s300_link_5"

        # Rutas
        self.path = "/home/user/Documentos/ROS/projects/kinova_robot_ws/src/kinova-ros/calibration/config"

        # Cargar calibracion hand-eye
        self.T_cam2ee = self.load_handeye_calibration()

        # Cargar calibracion intrinseca de la camara
        data = np.load(f"{self.path}/camera_final_calibration.npz")
        self.camera_matrix = data['camera_matrix']
        self.dist_coeffs   = data['dist_coeffs']

        # Buffer TF
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        self.bridge = CvBridge()

        # Configuracion ArUco
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
        self.parameters = cv2.aruco.DetectorParameters()
        self.detector   = cv2.aruco.ArucoDetector(self.aruco_dict, self.parameters)

        self.marker_size = 0.045  # metros

        # Publicador del centro del ArUco en el marco de la base
        self.pub_center = rospy.Publisher("/aruco/center_base", PointStamped, queue_size=1)

        rospy.Subscriber("/camera/color/image_raw", Image, self.image_callback)

        rospy.loginfo("Detector de centro ArUco iniciado.")
        rospy.spin()


    def load_handeye_calibration(self):
        try:
            data = np.load(f"{self.path}/handeye_result.npz")
            T = data['T']
            rospy.loginfo("Calibracion hand-eye cargada desde handeye_result.npz")
            return T
        except Exception:
            pass

        # Si no hay .npz, cargar desde YAML y reconstruir la matriz
        try:
            with open(f"{self.path}/handeye_calibration.yaml", "r") as f:
                cal = yaml.safe_load(f)

            tx = cal['translation']['x']
            ty = cal['translation']['y']
            tz = cal['translation']['z']
            qx = cal['rotation']['x']
            qy = cal['rotation']['y']
            qz = cal['rotation']['z']
            qw = cal['rotation']['w']

            T = tf_conversions.transformations.quaternion_matrix([qx, qy, qz, qw])
            T[0:3, 3] = [tx, ty, tz]

            rospy.loginfo("Calibracion hand-eye cargada desde handeye_calibration.yaml")
            return T

        except Exception as e:
            rospy.logerr(f"No se pudo cargar la calibracion hand-eye: {e}")
            raise


    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

            corners, ids, _ = self.detector.detectMarkers(gray)

            if ids is not None:
                rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(
                    corners, self.marker_size, self.camera_matrix, self.dist_coeffs)

                for i, (r, t, corner) in enumerate(zip(rvec, tvec, corners)):

                    # --- Centro del ArUco en coordenadas de la camara ---
                    center_cam = t.reshape(3, 1)
                    center_cam_h = np.vstack([center_cam, [1.0]])

                    # --- Transformar al marco del end effector ---
                    center_ee_h = self.T_cam2ee @ center_cam_h
                    center_ee   = center_ee_h[0:3].reshape(3)

                    # --- Transformar al marco de la base ---
                    center_base = self.transform_to_base(center_ee)

                    if center_base is not None:
                        self.publish_center(center_base, msg.header.stamp)
                        rospy.loginfo_throttle(1.0,
                            f"[ID {ids[i][0]}] Centro en base -> "
                            f"x={center_base[0]:.4f}  y={center_base[1]:.4f}  z={center_base[2]:.4f} m")

                    # --- Visualizacion en imagen ---
                    cv2.aruco.drawDetectedMarkers(cv_image, [corner], ids[i:i+1])
                    cv2.drawFrameAxes(cv_image, self.camera_matrix, self.dist_coeffs,
                                      r, t, self.marker_size)

                    pixel_center = self.project_center_to_image(center_cam)
                    if pixel_center is not None:
                        cx, cy = pixel_center
                        cv2.circle(cv_image, (cx, cy), 6, (0, 0, 255), -1)
                        cv2.putText(cv_image,
                                    f"ID:{ids[i][0]}",
                                    (cx + 10, cy - 10),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

                        # label se define ANTES de usarse — fix: label referenced before assignment
                        if center_base is not None:
                            label = (f"x={center_base[0]:.3f} "
                                     f"y={center_base[1]:.3f} "
                                     f"z={center_base[2]:.3f}")
                            # Sombra negra para contraste
                            cv2.putText(cv_image, label,
                                        (cx + 11, cy + 16),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 3)
                            # Azul clarito encima
                            cv2.putText(cv_image, label,
                                        (cx + 10, cy + 15),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 200, 50), 2)
            else:
                cv2.putText(cv_image, "ArUco no detectado", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

            cv2.imshow("ArUco Center Detector", cv_image)
            cv2.waitKey(1)

        except Exception as e:
            rospy.logwarn(f"Error en image_callback: {e}")


    def transform_to_base(self, center_ee):
        """Transforma un punto del marco del EE al marco de la base usando TF"""
        try:
            trans = self.tfBuffer.lookup_transform(
                self.base_frame,
                self.ee_frame,
                rospy.Time(0),
                rospy.Duration(0.1)
            )

            t = trans.transform.translation
            q = trans.transform.rotation

            T_base_ee = tf_conversions.transformations.quaternion_matrix(
                [q.x, q.y, q.z, q.w])
            T_base_ee[0:3, 3] = [t.x, t.y, t.z]

            p_ee_h = np.array([center_ee[0], center_ee[1], center_ee[2], 1.0])
            p_base_h = T_base_ee @ p_ee_h

            return p_base_h[0:3]

        except Exception as e:
            rospy.logwarn_throttle(2.0, f"Error lookup TF base<-ee: {e}")
            return None


    def project_center_to_image(self, center_cam):
        """Proyecta el centro 3D del marcador al plano imagen (pixeles)"""
        try:
            pts = center_cam.reshape(1, 1, 3).astype(np.float32)
            projected, _ = cv2.projectPoints(pts, np.zeros(3), np.zeros(3),
                                             self.camera_matrix, self.dist_coeffs)
            cx = int(projected[0][0][0])
            cy = int(projected[0][0][1])
            return cx, cy
        except Exception:
            return None


    def publish_center(self, center_base, stamp):
        """Publica el centro del ArUco en el marco de la base como PointStamped"""
        msg = PointStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = self.base_frame
        msg.point.x = float(center_base[0])
        msg.point.y = float(center_base[1])
        msg.point.z = float(center_base[2])
        self.pub_center.publish(msg)


if __name__ == "__main__":
    ArucoCenterDetector()