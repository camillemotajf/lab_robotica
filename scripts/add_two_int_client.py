#!/usr/bin/env python

import sys
import rospy
from beginner_tutorials.srv import *

def add_two_int_client(x, y):
    rospy.wait_for_service("add_two_ints")
    try:
        add_two_int = rospy.ServiceProxy("add_two_ints", AddTwoInts)
        resp = add_two_int(x,y)
        return resp.sum
    except:
        print("Serviço fslhou")


if __name__ == "__main__":
    if (len(sys.argv) == 3):
        x = int(sys.argv[1])
        y = int(sys.argv[2])
    else:
        print("Número de parametros inválido")
        sys.exit(1)

    print("Requisitando: ", x, "+", y)
    add_two_int_client(x,y)
