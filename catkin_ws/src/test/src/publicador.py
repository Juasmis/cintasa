#!/usr/bin/python

import rospy
from std_msgs.msg import String


def publicador():

    pub     = rospy.Publisher('publicador', String, queue_size=1)

    rate    = rospy.Rate(1)

    mensaje = String()

    contador = 0
    while not rospy.is_shutdown():
        cadena_a_publicar = "Publicacion %d"%contador
        contador += 1

        mensaje.data = cadena_a_publicar
        pub.publish(mensaje)

        rospy.loginfo(cadena_a_publicar)

        rate.sleep()


if __name__ == "__main__":
    rospy.init_node("publicador")
    publicador()

