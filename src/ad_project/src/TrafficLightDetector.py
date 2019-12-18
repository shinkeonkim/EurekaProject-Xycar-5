import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class TrafficLightDetector:

    def __init__(self, topic):
        self.bridge = CvBridge()
        rospy.Subscriber(topic, Image, self.conv_image)

        self.cam_img = np.zeros((480, 640, 3))

    def conv_image(self, data):
        self.cam_img = self.bridge.imgmsg_to_cv2(data, 'bgr8')

    def detect(self):
        image = self.cam_img
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1, 10, param1=200, param2=40, minRadius=0, maxRadius=50)

        if circles is not None:
            circles = np.uint16(np.around(circles))
            mask = np.zeros(np.shape(gray), dtype=np.uint8)

            hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV_FULL)
            h, s, v = cv2.split(hsv)
            cand = []

            for i in circles[0, :]:
                cv2.circle(mask, (i[0], i[1]), i[2], 255, -1)

                try:
                    i = list(map(int, i))
                    m, M = (i[1] - i[2], i[0] - i[2]), (i[1] + i[2], i[0] + i[2])
                    circle = np.zeros((i[2] * 2, i[2] * 2), dtype=np.uint8)
                    cv2.circle(circle, (i[2], i[2]), i[2], 255, -1)
                    roi = h[m[0]:M[0], m[1]:M[1]] & cv2.inRange(s[m[0]:M[0], m[1]:M[1]], 50, 255) & circle
                    cnts = [np.count_nonzero(cv2.inRange(roi, l, u)) for l, u in [(10, 30), (40, 60), (60, 80)]]#(1, 10), (20, 30), (50, 70)]]
                    if max(map(lambda x: x / (i[2] ** 2 * np.pi), cnts)) < 0.7:
                        continue
                    b, g, r = cv2.split(image[m[0]:M[0], m[1]:M[1]])
                    b, g, r = b & circle, g & circle, r & circle
                    cand.append((cnts, cv2.merge((b, g, r))))
                except:
                    continue

            if len(cand) != 0:
                rog = [max(cand, key=lambda x: x[0][i]) for i in range(3)]
                M = 0

                for i in range(1, 3):
                    if rog[i][0][i] > rog[M][0][M]:
                        M = i

                return ['red', 'orange', 'green'][M]

        return None

