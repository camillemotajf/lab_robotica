#!/usr/bin/env python

import rospy
from beginner_tutorials.msg import Funcionarios

def callback(mensagem):
    print(f"O nome do funcionário é {mensagem.nome}, com idade: {mensagem.idade}, com cargo {mensagem.cargo} e altura {mensagem.altura}")

def ouvinte():
    rospy.init_node("aplicacao", anonymous=True)
    rospy.Subscriber('dados_funcionarios', Funcionarios, callback=callback)
    rospy.spin()

if __name__ == "__main__":
    ouvinte()