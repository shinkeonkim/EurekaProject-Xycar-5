#!/usr/bin/env python

import rospy, time
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

rospy.init_node('track_tester')
pub = rospy.Publisher('/usb_cam/image_raw',
                      Image,
                      queue_size=1)
cap = cv2.VideoCapture('/home/nvidia/xycar/src/auto_drive/src/2.avi')
bridge = CvBridge()
rate = rospy.Rate(15)

while not rospy.is_shutdown():
    ret, frame = cap.read()
    if not ret:
        break
    cv2.imshow('captured', frame)
    if cv2.waitKey(1) & 0xff == 27:
        break
    msg = bridge.cv2_to_imgmsg(frame, 'bgr8')
    pub.publish(msg)
    rate.sleep()

cap.release()
cv2.destroyAllWindows()

