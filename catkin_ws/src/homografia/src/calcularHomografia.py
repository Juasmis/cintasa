#!/usr/bin/python


import cv2
import numpy as np

# puntos origen
puntos_origen = np.array([[540, 800], [800, 800], [600, 90],[720, 90]])
# puntos destino
puntos_destino = np.array([[540, 800],[800, 800],[540, 90],[800, 90]])

img = cv2.imread("image3.png")
np_image = cv2.imdecode(img, cv2.IMREAD_COLOR)
plt.imshow(np_image)
plt.show()
# calculate matrix H
h, status = cv2.findHomography(puntos_origen, puntos_destino)
print(h)
# provide a point you wish to map from image 1 to image 2
#a = np.array([[154, 174]], dtype='float32')
#a = np.array([a])

# finally, get the mapping
imagen_transformada = cv2.perspectiveTransform(np_image, h)
plt.imshow(imagen_transformada)
plt.show()
