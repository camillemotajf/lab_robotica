#!/usr/bin/env python

import rospy
from beginner_tutorials.srv import AddTwoInts, AddTwoIntsResponse

def handle(req):
    print(f"Retornando os valores de soma de {req.a} e {req.b}")
    soma = req.a + req.b
    return AddTwoIntsResponse(soma)

def server():
    rospy.init_node('server')
    s = rospy.Service('add_two_ints', AddTwoInts, handle)
    print("Servidor Funcionando: Esperando requisição")
    rospy.spin()


if __name__ == "__main__":
    server()