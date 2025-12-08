#!/usr/bin/env python3
import rospy
import sys, select, termios, tty
from geometry_msgs.msg import Twist

# Velocidades padrão
LIN_VEL = 0.2
ANG_VEL = 0.5

class RosAriaTeleop:
    def __init__(self):
        rospy.init_node('rosaria_teleop_teclado')
        self.pub = rospy.Publisher('/RosAria/cmd_vel', Twist, queue_size=10)
        self.vel = Twist()
        self.settings = termios.tcgetattr(sys.stdin)

        rospy.loginfo("🎮 Controle manual do RosAria iniciado!")
        rospy.loginfo("Use as setas (↑ ↓ ← →) para mover, espaço para parar, e 'q' para sair.")

    def get_tecla(self):
        """Leitura não bloqueante do teclado."""
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        tecla = ''
        if rlist:
            tecla = sys.stdin.read(3)
            if tecla[0] != '\x1b':  # se for tecla normal (ex: espaço ou q)
                tecla = tecla[0]
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return tecla

    def parar(self):
        """Zera velocidades lineares e angulares."""
        self.vel.linear.x = 0.0
        self.vel.angular.z = 0.0
        self.pub.publish(self.vel)

    def run(self):
        rate = rospy.Rate(10)
        rodando = True

        while not rospy.is_shutdown() and rodando:
            tecla = self.get_tecla()

            if tecla == 'w':  # ↑ frente
                self.vel.linear.x = LIN_VEL
                self.vel.angular.z = 0.0
                rospy.loginfo("⬆️  Frente")

            elif tecla == 's':  # ↓ ré
                self.vel.linear.x = -LIN_VEL
                self.vel.angular.z = 0.0
                rospy.loginfo("⬇️  Ré")

            elif tecla == 'd':  # → direita
                self.vel.linear.x = 0.0
                self.vel.angular.z = -ANG_VEL
                rospy.loginfo("➡️  Girando à direita")

            elif tecla == 'a':  # ← esquerda
                self.vel.linear.x = 0.0
                self.vel.angular.z = ANG_VEL
                rospy.loginfo("⬅️  Girando à esquerda")

            elif tecla == ' ':  # espaço → para, mas continua no loop
                self.parar()
                rospy.loginfo("⏸️  Parado (pressione seta para continuar)")

            elif tecla == 'q':  # sai do programa
                rospy.loginfo("👋 Encerrando teleop...")
                self.parar()
                rodando = False
                break

            if tecla:
                self.pub.publish(self.vel)

            rate.sleep()

        # restaura o terminal
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)


if __name__ == '__main__':
    try:
        teleop = RosAriaTeleop()
        teleop.run()
    except rospy.ROSInterruptException:
        pass
