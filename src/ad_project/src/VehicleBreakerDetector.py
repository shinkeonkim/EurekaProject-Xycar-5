import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class VehicleBreakerDetector:

    def __init__(self, topic):
        self.bridge = CvBridge()
        rospy.Subscriber(topic, Image, self.conv_image)

        self.cam_img = np.zeros((480, 640, 3))

    def conv_image(self, data):
        self.cam_img = self.bridge.imgmsg_to_cv2(data, 'bgr8')

    def detect(self):
        ret = False
        img_color = self.cam_img
        height,width = img_color.shape[:2]

        img_hsv = cv2.cvtColor(img_color, cv2.COLOR_BGR2HSV)


        lower_blue = (90, 130, 130)
        upper_blue = (130, 255, 255)
    
        img_mask = cv2.inRange(img_hsv, lower_blue, upper_blue)

        cnt = cv2.countNonZero(img_mask)

        #img_result = cv2.bitwise_and(img_color, img_color, mask = img_mask)
        #print(cnt)

        #cv2.imshow('img_color', img_color)
        #cv2.imshow('img_mask', img_mask)
        #cv2.imshow('img_result', img_result)  
        if cnt >= 13000:
            ret = True

        return (ret,cnt)

