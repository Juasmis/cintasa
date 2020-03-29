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

ancho_imagen = 640
alto_imagen = 480

#h = np.matrix([ 2.54269972e+00, 1.25619835e+00,-1.00495868e+03]
# [ 3.70286656e-16, 2.71625344e+00,-1.38842975e+02]
# [ 6.54819630e-19, 1.92837466e-03, 1.00000000e+00]])

#h = np.matrix('[3.70286656e-16,2,  3; 4,5,6 ; 7,8,9]', dtype=np.float32)
h = np.matrix('[2.54269972 , 1.25619835, -1.00495868e+03; 3.70286656e-16, 2.71625344, -1.38842975e+02; 6.54819630e-19, 1.92837466e-03, 1.0]', dtype=np.float32)
print(h)
# -----------------------------------------------

class nodoHomografia(object):
    def __init__(self):

        # Se publica en:
        self.pub_imagen_transformada = rospy.Publisher("/imagen_transformada/image_raw/compressed", CompressedImage, queue_size = 1)
        # self.pub_imagen_procesada = rospy.Publisher("/imagen_procesada/image_raw/compressed", CompressedImage, queue_size = 1)

        # Se suscribe a:
        self.sub_image = rospy.Subscriber("/raspicam_node/image/compressed", CompressedImage, self.inicioProcesamiento, queue_size=1)


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
        #
        # # Crear y pulicar imagen procesada
        # msg = CompressedImage()
        # msg.header.stamp = rospy.Time.now()
        # msg.format = "jpeg"
        # msg.data = np.array(cv2.imencode('.jpg', np_image)[1]).tostring()
        #
        # self.pub_imagen_procesada.publish(msg)


         # Transformacion de la imagen
        imagen_transformada = cv2.warpPerspective(np_image, h, (np_image.shape[1],np_image.shape[0]))

        #imagen_transformada = cv2.resize(np_image, (ancho_imagen, alto_imagen), interpolation = cv2.INTER_AREA)
        #imagen_transformada = np_image[int(0.15*alto_imagen):int(0.15*alto_imagen+0.6*alto_imagen), int(0.45*ancho_imagen):int(0.45*ancho_imagen+0.3*ancho_imagen)]


        # Crear y pulicar imagen corregida
        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = "jpeg"
        msg.data = np.array(cv2.imencode('.jpg', imagen_transformada)[1]).tostring()

        self.pub_imagen_transformada.publish(msg)

if __name__ == '__main__':
    rospy.init_node('nodoHomografia')
    nodoHomografia = nodoHomografia()
    rospy.loginfo("Iniciando nodo corrector de perspectiva")
    rospy.loginfo("La matriz de transformacion usada es:")
    rospy.loginfo(h)
    try:
        rospy.loginfo("Hola")
        rospy.spin()
    except KeyboardInterrupt:
        print("Cerrando..")
