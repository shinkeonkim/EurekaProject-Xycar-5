#!/usr/bin/env python

import rospy, time
import math

from linedetector import LineDetector
from obstacledetector import ObstacleDetector
from motordriver import MotorDriver

class AutoDrive:

    def __init__(self):
        rospy.init_node('xycar_driver')
        self.line_detector = LineDetector('/usb_cam/image_raw')
        self.obstacle_detector = ObstacleDetector('/ultrasonic')
        self.driver = MotorDriver('/xycar_motor_msg')

    def trace(self):
        obs_l, obs_m, obs_r = self.obstacle_detector.get_distance()
        line_theta,left,right = self.line_detector.detect_lines()
        angle = self.steer(line_theta,left,right)
        speed = self.accelerate(angle,line_theta,left,right)
        #print(line_theta,angle,speed)
        print(line_theta, left, right)
        self.driver.drive(angle + 90 + 1.8, speed)

    def steer(self, theta, left, right):
        """
        weight = 0.0

        if left == -1:
            weight = 0.0

        elif right == -1:
            weight = 0.0
        else:
            mid = (left + right) / 2
            diff = 55-mid
            
            if abs(diff) < 3:
                weight = 0.0
            # car is at right
            elif diff < 0:
                weight = -1.0
            elif diff > 0:
                weight = 1.0
        """
        K = 0.0
        if -0.8 <= theta <= 0.8:
            K = 8.0
        elif -4 <= theta <= 4:
            K = 2.5
        elif -15 <= theta <= 15:
            if theta < 0:
                if theta > -10:
                    K = 1.86
                else:
                    K = 1.4
            else:
                K = 1.75
        elif theta >= 20:
            K = 3.0
        elif theta <= -20:
            K = 2.1
        else:
            K = 1.5	
        """
        if theta > 0:
            K = 1.75
        else:
            K = 1.6
        """
        angle = theta * K

        return angle #+ (weight * 15)
        #return angle

    def accelerate(self, angle, theta, left, right):
        K = 130

        speed = K - min(abs(theta)/2, 15) 

        return speed

    def exit(self):
        print('finished')

if __name__ == '__main__':
    car = AutoDrive()
    time.sleep(3)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        car.trace()
        rate.sleep()
    rospy.on_shutdown(car.exit)
