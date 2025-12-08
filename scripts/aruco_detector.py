#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge

class ArucoDetectorNode:
    def __init__(self):
        rospy.init_node("aruco_detector", anonymous=False)

        self.bridge = CvBridge()

        # Mesma configuração do seu sistema standalone
        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        self.parameters = cv2.aruco.DetectorParameters_create()

        # Subscriber da câmera
        self.image_sub = rospy.Subscriber(
            "/usb_cam/image_raw",
            Image,
            self.image_callback,
            queue_size=1
        )

        # Publicação da pose
        self.pose_pub = rospy.Publisher(
            "/aruco_pose",
            PoseStamped,
            queue_size=10
        )

        # Publicação da imagem com marcação
        self.debug_pub = rospy.Publisher(
            "/aruco_detector/debug_image",
            Image,
            queue_size=1
        )

        rospy.loginfo("Aruco detector iniciado.")

    def image_callback(self, msg):
        # Converter imagem ROS → OpenCV
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logerr(f"Erro ao converter imagem: {e}")
            return

        corners, ids, rejected = cv2.aruco.detectMarkers(
            frame,
            self.dictionary,
            parameters=self.parameters
        )

        debug_frame = frame.copy()

        if ids is not None:
            cv2.aruco.drawDetectedMarkers(debug_frame, corners, ids)

            for i in range(len(ids)):
                corner = corners[i][0]

                # Centro
                cx = int(np.mean(corner[:, 0]))
                cy = int(np.mean(corner[:, 1]))

                # Vetor do canto 0 para o canto 1
                v = corner[1] - corner[0]
                theta_rad = np.arctan2(v[1], v[0])
                theta_deg = theta_rad * 180.0 / np.pi

                # Desenhar centro
                cv2.circle(debug_frame, (cx, cy), 5, (0, 255, 0), -1)

                # Escrever texto igual ao seu código
                text = f"ID: {ids[i][0]}, x: {cx}, y: {cy}, theta: {theta_deg:.2f}"
                cv2.putText(
                    debug_frame,
                    text,
                    (cx, cy - 40),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 0),
                    2
                )

                # Publicar pose
                pose = PoseStamped()
                pose.header.stamp = rospy.Time.now()
                pose.header.frame_id = "camera"

                pose.pose.position.x = float(cx)
                pose.pose.position.y = float(cy)
                pose.pose.position.z = 0.0

                pose.pose.orientation.x = 0.0
                pose.pose.orientation.y = 0.0
                pose.pose.orientation.z = np.sin(theta_rad / 2)
                pose.pose.orientation.w = np.cos(theta_rad / 2)

                self.pose_pub.publish(pose)

        # Publicar debug image
        try:
            debug_msg = self.bridge.cv2_to_imgmsg(debug_frame, encoding="bgr8")
            self.debug_pub.publish(debug_msg)
        except Exception as e:
            rospy.logerr(f"Erro ao converter imagem debug: {e}")


if __name__ == "__main__":
    ArucoDetectorNode()
    rospy.spin()
