#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
import os
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class GazeboCalibrator:
    def __init__(self):
        rospy.init_node('gazebo_camera_calibrator', anonymous=True)
        self.bridge = CvBridge()
        
        # Parámetros del patrón
        self.pattern_size = (8, 6)
        self.square_size = 0.025 # Recomendado usar metros (25mm = 0.025m)
        self.path = "/home/user/Documentos/PP/robot_ws/src/sim_kinova/config"

        if not os.path.exists(self.path):
            os.makedirs(self.path)

        # Preparar puntos 3D
        self.objp = np.zeros((self.pattern_size[0] * self.pattern_size[1], 3), np.float32)
        self.objp[:, :2] = np.mgrid[0:self.pattern_size[0], 0:self.pattern_size[1]].T.reshape(-1, 2)
        self.objp *= self.square_size

        self.objpoints = []
        self.imgpoints = []
        self.current_image = None
        self.last_gray_shape = None

        self.image_sub = rospy.Subscriber("/d415/color/image_raw", Image, self.image_callback)
        
        print(f"--- Nodo Iniciado ---")
        print("INSTRUCCIONES:")
        print("1. Presiona [ENTER] para capturar pose.")
        print("2. Presiona [Q] para CALCULAR Y GUARDAR la matriz.")

    def image_callback(self, data):
        try:
            self.current_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)

    def run(self):
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            if self.current_image is None:
                continue

            display = self.current_image.copy()
            gray = cv2.cvtColor(display, cv2.COLOR_BGR2GRAY)
            self.last_gray_shape = gray.shape[::-1]

            found, corners = cv2.findChessboardCorners(gray, self.pattern_size, None)

            if found:
                # Refinar esquinas para mayor precisión
                criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
                corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
                
                cv2.drawChessboardCorners(display, self.pattern_size, corners2, found)
                cv2.putText(display, f"Capturas: {len(self.objpoints)} - [ENTER]", (20, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            else:
                cv2.putText(display, "Buscando patron...", (20, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

            cv2.imshow("Calibracion en Gazebo", display)
            key = cv2.waitKey(1) & 0xFF

            if key == 13: # ENTER
                if found:
                    self.objpoints.append(self.objp.copy())
                    self.imgpoints.append(corners2)
                    print(f"[OK] Captura {len(self.objpoints)} guardada.")
                else:
                    print("[!] No se detecta el patrón.")

            elif key == ord('q') or key == ord('Q'):
                break
            
            rate.sleep()

        self.save_results()

    def save_results(self):
        cv2.destroyAllWindows()
        if len(self.objpoints) >= 2: # Se recomiendan al menos 10 para una buena matriz
            print("\nProcesando calibracion (esto puede tardar)...")
            
            # CALCULO DE LA MATRIZ DE CALIBRACION
            ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
                self.objpoints, self.imgpoints, self.last_gray_shape, None, None
            )

            if ret:
                filename = os.path.join(self.path, "camera_calibration.npz")
                # Guardamos con los nombres exactos que busca el script de Hand-Eye
                np.savez(filename, 
                         objpoints=self.objpoints,
                         imgpoints=self.imgpoints,
                         camera_matrix=mtx, 
                         dist_coeffs=dist, 
                         image_shape=self.last_gray_shape)
                
                print("\n" + "="*40)
                print(f"MATRIZ CALCULADA Y GUARDADA EN: {filename}")
                print(f"Error de reproyección: {ret:.4f}")
                print("="*40)
            else:
                print("[ERROR] No se pudo calibrar la cámara.")
        else:
            print(f"\n[!] Necesitas más capturas (tienes {len(self.objpoints)}, mínimo 2).")

if __name__ == '__main__':
    calibrator = GazeboCalibrator()
    try:
        calibrator.run()
    except rospy.ROSInterruptException:
        pass