#!/usr/bin/env python

import rospy
from beginner_tutorials.msg import Funcionarios

def talker():
    pub = rospy.Publisher('dados_funcionarios', Funcionarios, queue_size=10)
    rospy.init_node('banco_de_dados', anonymous=True)
    rate = rospy.Rate(10)

    nomes = ['Iago', 'Andre']
    nascimentos = [29, 50]
    cargos = ['Professor', 'Professor']
    alturas = [1.81, 1.71]

    for i in range(len(nomes)):
        perfil = Funcionarios()
        perfil.nome = nomes[i]
        perfil.idade = nascimentos[i]
        perfil.cargo = cargos[i]
        perfil.altura = alturas[i]
        pub.publish(perfil)
        rate.sleep()

if __name__ == "__main__":
    try:
        talker()
    except rospy.ROSInterruptException:
        pass