import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class BusStopDetector:
    def __init__(self, topic):
        self.bridge = CvBridge()
        rospy.Subscriber(topic, Image, self.conv_image)

        self.cam_img = np.zeros((480, 640, 3))
        template_img = cv2.imread('/home/nvidia/template.jpg')
        self.template = max(self.find_circles(template_img), key=lambda x: x[0].shape[0] * x[0].shape[1])[0]
        self.body_cascade = cv2.CascadeClassifier('/home/nvidia/haarcascade_fullbody.xml')

    def conv_image(self, data):
        self.cam_img = self.bridge.imgmsg_to_cv2(data, 'bgr8')

    def find_circles(self, image):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1, 10, param1=200, param2=70, minRadius=10, maxRadius=100)
        results = []

        if circles is not None:
            circles = np.uint16(np.around(circles))
            hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV_FULL)

            for i in circles[0, :]:
                try:
                    i = list(map(lambda x: int(round(x)), i))
                    m, M = (i[1] - i[2], i[0] - i[2]), (i[1] + i[2], i[0] + i[2])
                    circle = np.zeros((i[2] * 2, i[2] * 2), dtype=np.uint8)
                    cv2.circle(circle, (i[2], i[2]), i[2], 255, -1)
                    roi = cv2.inRange(hsv[m[0]:M[0], m[1]:M[1]], np.array([0, 0, 200]), np.array([255, 50, 255]))
                    roi &= circle
                    results.append((roi, m, M))
                except:
                    pass

        return results

    def detect(self):
        circles = self.find_circles(self.cam_img)
        if circles:
            minDiff, minRoi, min_, min_data = None, None, np.inf, None
            for circle_data in circles:
                circle = circle_data[0]
                diff = cv2.absdiff(cv2.resize(circle, self.template.shape[:2]), self.template)
                diffVal = np.sum(diff)
                if diffVal < min_:
                    minDiff = diff
                    minRoi = circle
                    min_ = diffVal
                    min_data = circle_data
            if minDiff is not None:
                cv2.waitKey(1)
                body_img = self.cam_img[circle_data[2][0]:min(circle_data[2][0]+120, self.cam_img.shape[0] - 1), max(circle_data[1][1]-100, 0):min(circle_data[1][1]+100, self.cam_img.shape[1] - 1)].copy()
                #small = cv2.resize(self.cam_img, (60, 100))
                if body_img is not None and body_img.shape[0] > 0 and body_img.shape[1] > 0:
                    gray = cv2.cvtColor(body_img, cv2.COLOR_BGR2GRAY)
                    thresh = cv2.Canny(gray, 100, 200)
                    '''
                    contours = cv2.findContours(thresh, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
                    mask = np.zeros(gray.shape, dtype=gray.dtype)
                    for contour in contours[1:-1]:
                        peri = cv2.arcLength(contour[0], True)
                        approx = cv2.approxPolyDP(contour[0], 0.02 * peri, True)
                        if len(approx) == 4:
                            cv2.fillPoly(mask, [approx], (255, 255, 255))
                        #cv2.drawContours(body_img, [approx], -1, (0, 255, 0), 3)
                    gray &= mask
                    '''
                    gray = ~gray
                    gray &= cv2.inRange(gray, 150, 255)
                    gray = ~gray
                    body = self.body_cascade.detectMultiScale(gray, 1.01, 2)
                    for (x,y,w,h) in body :
                        #x = x * body_img.shape[1] // small.shape[1]
                        #y = y * body_img.shape[0] // small.shape[0]
                        #w = w * body_img.shape[1] // small.shape[1]
                        #h = h * body_img.shape[0] // small.shape[0]
                        #x, y = x - (m[1] + 80), y - M[0]
                        cv2.rectangle(body_img,(x,y),(x+w,y+h),(0,0,255),3)

                    return min_ < 200000, (min_data[1], min_data[2]), len(body)
                return min_ < 200000, (min_data[1], min_data[2]), 0
        return False, ((-1, -1), (-1, -1)), -1
