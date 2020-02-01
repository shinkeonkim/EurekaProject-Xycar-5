import rospy
import cv2
import numpy as np
import math
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class LineDetector:

    def __init__(self, topic):
        # Initialize various class-defined attributes, and then...
        self.cam_img = np.zeros(shape=(480, 640, 3), dtype=np.uint8)
        self.vid_out = cv2.VideoWriter('/home/nvidia/output.avi', cv2.VideoWriter_fourcc(*'MJPG'), 30, (640, 480))
        '''
        self.mask = np.zeros(shape=(self.scan_height, self.image_width),
                             dtype=np.uint8)
        self.edge = np.zeros(shape=(self.scan_height, self.image_width),
                             dtype=np.uint8)
        '''
        self.bridge = CvBridge()
        rospy.Subscriber(topic, Image, self.conv_image)
        self.before = np.array([[0, 315], [639, 305], [170, 260], [460, 250]], dtype='float32')
        self.after = np.array([[0, 100], [100, 100], [0, -100], [100, -100]], dtype='float32')
        self.theta = 0.0

    def __del__(self):
        self.vid_out.release()

    def conv_image(self, data):
        self.cam_img = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        self.vid_out.write(self.cam_img)
        '''
        v = self.roi_vertical_pos
        roi = self.cam_img[v:v + self.scan_height, :]

        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        avg_value = np.average(hsv[:, :, 2])
        value_threshold = avg_value * 1.0
        lbound = np.array([0, 0, value_threshold], dtype=np.uint8)
        ubound = np.array([100, 255, 255], dtype=np.uint8)
        self.mask = cv2.inRange(hsv, lbound, ubound)

        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        self.edge = cv2.Canny(blur, 60, 70)
        '''

    def detect_lines(self):
        #self.theta = 0.0
        frame = self.cam_img
        tdSize = (100, 210)
        m = cv2.getPerspectiveTransform(self.before, self.after)
        topdown = cv2.warpPerspective(frame, m, tdSize)

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (3, 3), 0)
        edges = cv2.Canny(gray, 50, 150, apertureSize=3)
        edges = cv2.warpPerspective(edges, m, tdSize)
        lines = cv2.HoughLines(edges, 1, np.pi / 180, 80)
        
        left, right = -1, -1
        #lCnt, rCnt = 0, 0
        
        if lines is not None:
            cnts = [0 for _ in range(50)]
            vals = [0.0 for _ in range(len(cnts))]
            cnt = 0

            ptCnts = [0 for _ in range(10)]
            ptVals = [0.0 for _ in range(len(ptCnts))]

            for line in lines:
                for rho, theta in line:
                    index = int(theta / np.pi * len(cnts))
                    cnts[index] += 1
                    vals[index] += theta
                    cnt += 1

                    a = np.cos(theta)
                    b = np.sin(theta)
                    x0 = a * rho
                    y0 = b * rho
                    '''
                    x1 = int(x0 + 1000 * (-b))
                    y1 = int(y0 + 1000 * (a))
                    x2 = int(x0 - 1000 * (-b))
                    y2 = int(y0 - 1000 * (a))

                    cv2.line(topdown, (x1, y1), (x2, y2), (0, 0, 255), 2)
                    '''
                    t = (199 - y0) / a
                    x = x0 - t * b

                    index = min(max(int(x / tdSize[0] * len(ptCnts)), 0), len(ptCnts) - 1)
                    ptCnts[index] += 1
                    ptVals[index] += x
                    '''
                    if x < 50:
                        left += x
                        lCnt += 1
                    else:
                        right += x
                        rCnt += 1
                    '''
            if cnt > 0:
                thetas = []

                for i in range(2):
                    index = cnts.index(max(cnts))
                    if cnts[index] == 0:
                        break
                    theta = vals[index] / cnts[index]
                    if theta > math.pi / 2:
                        theta -= math.pi
                    thetas.append(theta)
                    cnts[index] = 0
                theta = sum(thetas) / len(thetas)

                #weight = 0.2
                self.theta = theta #self.theta * (1.0 - weight) + theta * weight

                pts = []
                half = len(ptCnts) // 2

                for cnts, vals in [(ptCnts[:half], ptVals[:half]), (ptCnts[half:], ptVals[half:])]:
                    index = cnts.index(max(cnts))
                    pt = vals[index] / cnts[index] if cnts[index] > 0 else -1
                    pts.append(pt)

                left, right = pts
                '''
                a = np.cos(theta)
                b = np.sin(theta)
                x0 = 50
                y0 = 100
                x1 = int(x0 + 1000 * (-b))
                y1 = int(y0 + 1000 * (a))
                x2 = int(x0 - 1000 * (-b))
                y2 = int(y0 - 1000 * (a))

                cv2.line(topdown, (x1, y1), (x2, y2), (0, 255, 0), 5)
                '''
        else:
            self.theta *= 0.915
        '''
        cv2.imshow("origin", frame)
        cv2.imshow('hough', topdown)
        cv2.waitKey(1)
        '''

        #left = left / lCnt if lCnt > 0 else -1
        #right = right / rCnt if rCnt > 0 else -1
        '''
	if left >= 0:
            cv2.circle(topdown, (int(left), 199), 3, (0, 0, 255), 3)

        if right >= 0:
            cv2.circle(topdown, (int(right), 199), 3, (255, 0, 0), 3)

        cv2.imshow('hough', topdown)
        #cv2.waitKey(1)
        
        frame = cv2.circle(frame, (0, 315), 2, (0, 255, 0), 3)
        frame = cv2.circle(frame, (639, 320), 2, (0, 255, 0), 3)

        frame = cv2.circle(frame, (170, 260), 2, (0, 255, 0), 3)
        frame = cv2.circle(frame, (460, 250), 2, (0, 255, 0), 3)
        cv2.imshow('origin', frame)
        cv2.imshow('edges', edges)
        cv2.waitKey(1)
        '''
        return self.theta * 180 / math.pi, left, right

    def show_images(self, left, right):
        # Display images for debugging purposes;
        # do not forget to call cv2.waitKey().
        pass
