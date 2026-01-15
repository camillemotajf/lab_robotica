#!/usr/bin/env python3
import rospy
import math
import time
from geometry_msgs.msg import PoseStamped, Twist

class PIDController:
    def __init__(self, kp, ki, kd, output_limit):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limit = output_limit

        self.prev_error = 0.0
        self.integral = 0.0
        self.last_time = None

    def compute(self, error):
        current_time = time.time()

        if self.last_time is None:
            self.last_time = current_time
            return 0.0

        dt = current_time - self.last_time
        if dt <= 0: return 0.0

        # Termo Proporcional
        P = self.kp * error

        # Termo Integral (com proteção anti-windup básica)
        self.integral += error * dt
        # Limita o acúmulo da integral para não saturar
        self.integral = max(min(self.integral, self.output_limit), -self.output_limit)
        I = self.ki * self.integral

        # Termo Derivativo
        derivative = (error - self.prev_error) / dt
        D = self.kd * derivative

        # Atualiza variáveis para próxima iteração
        self.prev_error = error
        self.last_time = current_time

        output = P + I + D

        # Saturação da saída final
        return max(min(output, self.output_limit), -self.output_limit)

    def reset(self):
        self.prev_error = 0.0
        self.integral = 0.0
        self.last_time = time.time()


class RobotPositioningSystem:
    def __init__(self):
        rospy.init_node('robot_positioning_node')

        # === CONFIGURAÇÕES DE ALVO ===
        self.target_distance = 0.50  # 50 cm do alvo
        self.tolerance_dist = 0.02  # 2 cm de erro aceitável
        self.tolerance_angle = 0.03  # ~1.7 graus de erro aceitável

        # === CONFIGURAÇÃO DOS PIDs ===
        # Linear: Kp(0.4), Ki(0.05), Kd(0.1) -> Ajuste conforme o peso do robô
        self.linear_pid = PIDController(kp=0.4, ki=0.01, kd=0.05, output_limit=0.4)

        # Angular: Kp(1.2), Ki(0.0), Kd(0.1) -> Rotação precisa responder rápido
        self.angular_pid = PIDController(kp=1.2, ki=0.0, kd=0.1, output_limit=0.8)

        # === SETUP ROS ===
        self.cmd_vel_pub = rospy.Publisher('/RosAria/cmd_vel', Twist, queue_size=1)
        self.pose_sub = rospy.Subscriber('/aruco_pose', PoseStamped, self.pose_callback)

        self.last_msg_time = rospy.Time.now()
        self.current_pose = None
        self.is_positioned = False

        rospy.loginfo("RobotPositioningSystem iniciado. Algoritmo PID ativo.")

        # Loop de controle principal (10Hz)
        self.rate = rospy.Rate(10)
        self.control_loop()

    def pose_callback(self, msg):
        self.current_pose = msg
        self.last_msg_time = rospy.Time.now()

    def control_loop(self):
        while not rospy.is_shutdown():
            # 1. Verificação de Segurança (Watchdog)
            # Se não receber dados por 0.5s, para o robô
            if (rospy.Time.now() - self.last_msg_time).to_sec() > 0.5:
                self.stop_robot()
                self.linear_pid.reset()  # Reseta integrais para não dar tranco quando voltar
                self.angular_pid.reset()
                rospy.logwarn_throttle(2, "Sem sinal visual. Parando.")
                self.rate.sleep()
                continue

            if self.current_pose:
                # 2. Extrair dados
                # Câmera frame: Z = frente, X = direita/esquerda
                curr_x = self.current_pose.pose.position.x
                curr_z = self.current_pose.pose.position.z

                # 3. Calcular Erros
                # Erro Distância: Quero chegar em target_distance. Se Z=1.0 e Target=0.5 -> Erro=0.5
                dist_error = curr_z - self.target_distance

                # Erro Angular: atan2(x, z) dá o ângulo exato para o centro
                angle_error = math.atan2(curr_x, curr_z)

                # 4. Verificar se já chegamos (Deadband)
                aligned = abs(angle_error) < self.tolerance_angle
                at_distance = abs(dist_error) < self.tolerance_dist

                if aligned and at_distance:
                    if not self.is_positioned:
                        rospy.loginfo("ALVO ALCANÇADO! Mantendo posição.")
                        self.is_positioned = True
                    self.stop_robot()

                else:
                    self.is_positioned = False

                    # 5. Calcular PID
                    # Angular: Invertemos o sinal dependendo da montagem (testar)
                    # Se X positivo (direita), robô deve virar direita (Z negativo no cmd_vel)
                    rot_vel = -self.angular_pid.compute(angle_error)

                    # Linear: O PID linear só atua forte se estivermos mais ou menos alinhados
                    # Isso evita que o robô corra para frente enquanto ainda está muito torto
                    if abs(angle_error) > 0.3:  # Se estiver muito torto (>17 graus)
                        lin_vel = 0.0  # Prioriza girar
                    else:
                        lin_vel = self.linear_pid.compute(dist_error)

                    self.publish_cmd(lin_vel, rot_vel)

                    # Debug Info
                    print(f"DistErr: {dist_error:.2f}m | AngErr: {angle_error:.2f}rad | "
                          f"CmdLin: {lin_vel:.2f} | CmdAng: {rot_vel:.2f}", end='\r')

            self.rate.sleep()

    def publish_cmd(self, linear, angular):
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.cmd_vel_pub.publish(msg)

    def stop_robot(self):
        self.publish_cmd(0.0, 0.0)


if __name__ == '__main__':
    try:
        RobotPositioningSystem()
    except rospy.ROSInterruptException:
        pass