#!/usr/bin/env python3

import numpy as np
import cv2
import os

path = "/home/user/Documentos/PP/robot_ws/src/sim_kinova/config"
data_file = os.path.join(path, "camera_calibration.npz")

# Verificar si el archivo existe
if not os.path.exists(data_file):
    print(f"Error: No se encontró el archivo en {data_file}")
    exit()

data = np.load(data_file, allow_pickle=True)

try:
    objpoints = data["objpoints"]
    imgpoints = data["imgpoints"]
    image_shape = tuple(data["image_shape"])
except KeyError as e:
    print(f"Error: El archivo .npz no tiene la llave {e}")
    print("Asegúrate de que el script de Gazebo guarde 'objpoints' e 'imgpoints'.")
    exit()

print(f"Número de imágenes válidas: {len(objpoints)}")

# Ejecutar calibración
ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
    objpoints,
    imgpoints,
    image_shape,
    None,
    None
)

# Mostrar resultados
print("\n" + "="*30)
print("=== MATRIZ INTRÍNSECA (K) ===")
print(camera_matrix)
print("\n=== COEFICIENTES DE DISTORSIÓN (D) ===")
print(dist_coeffs.ravel())
print("="*30)

# Calcular error de reproyeccion
mean_error = 0
for i in range(len(objpoints)):
    imgpoints2, _ = cv2.projectPoints(
        objpoints[i], rvecs[i], tvecs[i], camera_matrix, dist_coeffs
    )
    error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2) / len(imgpoints2)
    mean_error += error

total_error = mean_error / len(objpoints)
print(f"\nERROR DE REPROYECCIÓN PROMEDIO: {total_error:.6f} px")

if total_error < 0.5:
    print("Resultado: EXCELENTE")
elif total_error < 1.0:
    print("Resultado: BUENO")
else:
    print("Resultado: MALA CALIDAD (Captura más poses con ángulos variados)")

output_file = os.path.join(path, "camera_calibration.npz")

np.savez(
    output_file,
    camera_matrix=camera_matrix,
    dist_coeffs=dist_coeffs,
    image_shape=image_shape
)

print(f"\nArchivo listo para Hand-Eye en:\n{output_file}")