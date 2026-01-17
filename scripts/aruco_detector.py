#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
import tf2_ros
import tf.transformations as tf_trans

from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped, TransformStamped
from cv_bridge import CvBridge

CAMERA_MATRIX = np.array([
    [616.97, 0.0, 328.68],
    [0.0, 618.21, 236.6],
    [0.0, 0.0, 1.0],
])


DISTORTION_COEFFS = np.array([0.041, -0.122515, 0.006851, 0.009198, 0.0])

class ArucoDetectorNode:
    def __init__(self):
        rospy.init_node("aruco_detector_tf", anonymous=False)

        self.bridge = CvBridge()

        # Inicializa o Broadcaster do TF (Isso permite ver a tag no Rviz relativa à camera)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        # Configuração do Dicionário (Verifique se é DICT_4X4 ou DICT_6X6)
        # No seu script de referencia era 4x4, no atual era 6x6. Vou deixar 6x6.
        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        self.parameters = cv2.aruco.DetectorParameters_create()


        self.marker_length = rospy.get_param("~marker_length", 0.05)

        # Publishers
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.image_callback, queue_size=1)
        self.pose_pub = rospy.Publisher("/aruco_pose", PoseStamped, queue_size=10)
        self.debug_pub = rospy.Publisher("/aruco_detector/debug_image", Image, queue_size=1)

        rospy.loginfo(f"Detector iniciado. Tamanho do Marcador configurado: {self.marker_length}m")

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logerr(f"Erro CvBridge: {e}")
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = cv2.aruco.detectMarkers(gray, self.dictionary, parameters=self.parameters)

        debug_frame = frame.copy()

        if ids is not None:
            # Estima a pose (Posição e Rotação) de todos os marcadores
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners,
                self.marker_length,
                CAMERA_MATRIX,
                DISTORTION_COEFFS
            )

            # Desenha contornos
            cv2.aruco.drawDetectedMarkers(debug_frame, corners, ids)

            for i in range(len(ids)):
                marker_id = ids[i][0]

                # Vetores crus do OpenCV (Rodrigues e Translação)
                rvec = rvecs[i][0]
                tvec = tvecs[i][0]

                # 1. VISUALIZAÇÃO: Desenha eixos X, Y, Z no centro do marcador
                cv2.drawFrameAxes(debug_frame, CAMERA_MATRIX, DISTORTION_COEFFS, rvec, tvec, 0.05)

                # 2. TF: Publica a transformação Camera -> Marker
                # frame_id da msg geralmente é "usb_cam"
                self.publish_tf(rvec, tvec, marker_id, msg.header.stamp, msg.header.frame_id)

                # 3. TÓPICO: Publica PoseStamped (para controle ou logs)
                self.publish_pose_stamped(rvec, tvec, msg.header.stamp, msg.header.frame_id)

                # Texto de debug na tela
                cx, cy = int(corners[i][0][:, 0].mean()), int(corners[i][0][:, 1].mean())
                dist = np.linalg.norm(tvec)
                cv2.putText(debug_frame, f"ID:{marker_id} Z:{dist:.2f}m", (cx, cy + 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        # Publica imagem processada
        try:
            self.debug_pub.publish(self.bridge.cv2_to_imgmsg(debug_frame, "bgr8"))
        except:
            pass

    def publish_tf(self, rvec, tvec, marker_id, stamp, parent_frame):
        t = TransformStamped()

        t.header.stamp = stamp
        t.header.frame_id = parent_frame
        t.child_frame_id = f"aruco_marker_{marker_id}"

        # Translação (Metros)
        t.transform.translation.x = tvec[0]
        t.transform.translation.y = tvec[1]
        t.transform.translation.z = tvec[2]

        # Rotação (De Rodrigues para Quaternion)
        q = self.rvec_to_quaternion(rvec)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.tf_broadcaster.sendTransform(t)

    def publish_pose_stamped(self, rvec, tvec, stamp, frame_id):
        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = frame_id

        pose.pose.position.x = tvec[0]
        pose.pose.position.y = tvec[1]
        pose.pose.position.z = tvec[2]

        q = self.rvec_to_quaternion(rvec)
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]

        self.pose_pub.publish(pose)

    def rvec_to_quaternion(self, rvec):
        # Converte vetor de rotação (Rodrigues) para Matriz e depois para Quaternion
        rotation_matrix = cv2.Rodrigues(rvec)[0]
        matrix_4x4 = np.eye(4)
        matrix_4x4[:3, :3] = rotation_matrix
        return tf_trans.quaternion_from_matrix(matrix_4x4)


if __name__ == "__main__":
    try:
        ArucoDetectorNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass