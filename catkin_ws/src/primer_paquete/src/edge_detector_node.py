#!/usr/bin/python
from __future__ import print_function

import rospy
import cv2
import argparse
import sys
import numpy as np
import time
import roslib


import threading

from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage

# Parametros configuracion ----------------------
BLANCO_MIN = (30, 30, 70)
BLANCO_MAX = (60, 60, 260)

AMARILLO_MIN = (10, 150, 20)
AMARILLO_MAX = (40, 260, 300)

ROJO_MIN = (0, 100, 100)
ROJO_MAX = (100, 350, 200)

ROJO2_MIN = (100, 180, 50)
ROJO2_MAX = (200, 290, 250)

#-------------------------------------------------

class nodoFiltroCanny(object):

    def __init__(self):

        # Se publica en:
        self.pub_detecciones = rospy.Publisher("/imagen_salida/canny/image_raw/compressed", CompressedImage, queue_size = 1)
        self.pub_detecciones_color = rospy.Publisher("/imagen_salida/color/image_raw/compressed", CompressedImage, queue_size = 1)
        self.pub_imagen_procesada = rospy.Publisher("/imagen_procesada/image_raw/compressed", CompressedImage, queue_size = 1)
        #self.bridge = CvBridge()
        # Se suscribe en:
        self.sub_image = rospy.Subscriber("/imagen_transformada/image_raw/compressed", CompressedImage, self.inicioProcesamiento, queue_size=1)

        # Thread lock
        self.thread_lock = threading.Lock()

    def inicioProcesamiento(self, imagen_entrada):
        thread = threading.Thread(target=self.procesarImagen,args=(imagen_entrada,))
        thread.setDaemon(True)
        thread.start()

    def procesarImagen(self, imagen_entrada):
        if not self.thread_lock.acquire(False):
            #rospy.loginfo("Thread locked")
            # Return immediately if the thread is locked
            return

        try:
            self.procesarImagen_(imagen_entrada)
        finally:
            # Release the thread lock
            self.thread_lock.release()


    def procesarImagen_(self, imagen_entrada):
        np_array = np.fromstring(imagen_entrada.data, np.uint8)
        np_image = cv2.imdecode(np_array, cv2.IMREAD_COLOR)

        # Crear y pulicar imagen procesada
        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = "jpeg"
        msg.data = np.array(cv2.imencode('.jpg', np_image)[1]).tostring()

        self.pub_imagen_procesada.publish(msg)

        # Detecciones

        ## Deteccion de bordes ##
        imagen_bordes = cv2.Canny(np_image,100,200)

        # Crear y pulicar imagen de bordes
        msg2 = CompressedImage()
        msg2.header.stamp = rospy.Time.now()
        msg2.format = "jpeg"
        msg2.data = np.array(cv2.imencode('.jpg', imagen_bordes)[1]).tostring()

        self.pub_detecciones.publish(msg2)

        ## Deteccion de colores ##
        imagen_color = cv2.resize(np_image, (128, 96), interpolation = cv2.INTER_AREA)
        hsv_img = cv2.cvtColor(imagen_color, cv2.COLOR_RGB2HSV)
        #Se extraen por separado cada color en distintas mascaras
        blanco = cv2.inRange(hsv_img, BLANCO_MIN, BLANCO_MAX)
        amarillo = cv2.inRange(hsv_img, AMARILLO_MIN, AMARILLO_MAX)
        rojo1 = cv2.inRange(hsv_img, ROJO_MIN, ROJO_MAX)
        rojo2 = cv2.inRange(hsv_img, ROJO2_MIN, ROJO2_MAX)
        rojo = rojo1 + rojo2
        resultado = blanco + amarillo + rojo

        #Resultados
        resultado_amarillo = cv2.bitwise_and(imagen_color,imagen_color, mask=amarillo)
        resultado_blanco = cv2.bitwise_and(imagen_color,imagen_color, mask=blanco)
        resultado_rojo = cv2.bitwise_and(imagen_color,imagen_color, mask=rojo)
        resultado_final = cv2.bitwise_and(imagen_color,imagen_color, mask=resultado)

        # Crear y pulicar imagen de colores detectados
        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = "jpeg"
        msg.data = np.array(cv2.imencode('.jpg', resultado_final)[1]).tostring()

        self.pub_detecciones_color.publish(msg)



if __name__ == '__main__':
    rospy.init_node('detector_bordes')
    nodo_filtro_canny = nodoFiltroCanny()
    rospy.loginfo("Iniciando nodo filtro de Canny")
    try:
        rospy.loginfo("Hola")
        rospy.spin()
    except KeyboardInterrupt:
        print("Cerrando..")
    #cv2.destroyAllWindows()
