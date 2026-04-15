#!/usr/bin/env python3
import rospy 
import ros_numpy # Para convertir mensajes ROS a NumPy, necesario para procesar imágenes con OpenCV
import numpy as np
import cv2
from sensor_msgs.msg import Image # Para suscribirnos a los frames de la cámara
# Importamos el predictor específico de SAM 3 para conceptos
from ultralytics.models.sam import SAM3SemanticPredictor 

class Sam3VisionNode:
    def __init__(self):
        rospy.init_node('sam3_vision_node')

        #Configuración de SAM 3
        overrides = dict(
            conf=0.25, #Bajamos el umbral para no perdernos nada, aunque aumente falsos positivos
            task="segment", #Siempre segmentación, no clasificación
            mode="predict", #Modo predictivo, no entrenamiento
            model="../weights/sam3.pt",
            half=False #En CPU Ryzen U-Series, False es más estable que True (aunque más lento)
        )
        
        self.predictor = SAM3SemanticPredictor(overrides=overrides) # Cargamos el modelo una sola vez al iniciar el nodo
        self.sub = rospy.Subscriber("/camera/color/image_raw", Image, self.callback, queue_size=1) # Suscripción con cola de tamaño 1 para no saturar la memoria
        rospy.loginfo("SAM 3 (Segment Anything with Concepts) listo") # Mensaje de log para confirmar que el nodo está listo

    """ Callback que se ejecuta cada vez que llega un nuevo frame de la cámara """
    def callback(self, msg):
        try:
            frame = ros_numpy.numpify(msg) # Convertimos el mensaje ROS a un array de NumPy para procesarlo con OpenCV
            frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR) # Convertimos a BGR para que OpenCV lo procese correctamente
            # --- SAM 3 ---
            # Le pedimos directamente el concepto "bowl" o "food"
            self.predictor.set_image(frame_bgr) # Configuramos la imagen en el predictor
            results = self.predictor(text=["bowl", "food", "person"]) # Pedimos segmentación por concepto, no por caja delimitadora
            if len(results) > 0 and results[0].masks is not None: # Si SAM 3 nos devuelve máscaras, las procesamos
                output = frame_bgr.copy() # Copiamos el frame original para dibujar las máscaras encima
                # SAM 3 nos devolverá máscaras para cada concepto
                for i, mask_data in enumerate(results[0].masks.data):
                    mask = mask_data.cpu().numpy().astype(bool) # Convertimos la máscara a un array booleano para usarlo como índice
                    # Pintamos el bowl y la comida
                    color = [0, 255, 0] if i == 0 else [255, 0, 0]
                    output[mask] = output[mask] * 0.5 + np.array(color) * 0.5 # Mezclamos el color con la imagen original para que se vea la segmentación
                
                cv2.imshow("SAM 3: Detección por Concepto", output) # Mostramos el resultado en una ventana de OpenCV
                cv2.waitKey(1) # Necesario para que OpenCV actualice la ventana, aunque no esperamos una tecla específica
        except Exception as e:
            rospy.logerr(f"Error en SAM 3: {e}")

if __name__ == '__main__':
    node = Sam3VisionNode() 
    rospy.spin() # Mantenemos el nodo en ejecución hasta que se cierre manualmente o ocurra un error crítico