#!/usr/bin/env python

import rospy, time
import math
import time

from linedetector import LineDetector
from obstacledetector import ObstacleDetector
from motordriver import MotorDriver
from TrafficLightDetector import TrafficLightDetector
from VehicleBreakerDetector import VehicleBreakerDetector
from BusStopDetector import BusStopDetector

class AutoDrive:

    def __init__(self):
        rospy.init_node('xycar_driver')
        self.line_detector = LineDetector('/usb_cam/image_raw')
        self.traffic_light_detector = TrafficLightDetector('/usb_cam/image_raw')
        self.vehicle_breaker_detector = VehicleBreakerDetector('/usb_cam/image_raw')
        self.bus_stop_detector = BusStopDetector('/usb_cam/image_raw')
        self.obstacle_detector = ObstacleDetector('/ultrasonic')
        self.driver = MotorDriver('/xycar_motor_msg')
        self.slow_time = time.time()
        self.prev_dis = 100
        self.prev_l = []
        self.prev_m = []
        self.prev_r = []
        self.MAX_SIZE = 3
        self.bus_stop_time = time.time()
        self.bus_ignore_time = -1
        self.bus_data = []

    def average(self, L):
        if len(L) == 0:
            return 100
        return sum(L) / len(L)

    def trace(self):
        k = 500
        
        if len(self.bus_data) > 4:
            self.bus_data.pop(0)
        self.bus_data.append((self.bus_stop_detector.detect()))
        bus_stop, stop_m, stop_M, people_cnt = 0, [0, 0], [0, 0], 0 #self.bus_data[-1]
        for data in self.bus_data:
            bus_stop += 1 if data[0] else 0
            stop_m[0] += data[1][0][0]
            stop_m[1] += data[1][0][1]
            stop_M[0] += data[1][1][0]
            stop_M[1] += data[1][1][1]
            people_cnt += data[2]
        bus_stop = bus_stop > len(self.bus_data) // 2
        stop_m[0] //= len(self.bus_data)
        stop_m[1] //= len(self.bus_data)
        stop_M[0] //= len(self.bus_data)
        stop_M[1] //= len(self.bus_data)
        people_cnt /= len(self.bus_data)
        '''
        if bus_stop and (stop_M[0] - stop_m[0]) * (stop_M[1] - stop_m[1]) > (208 - 164) * (100 - 56):
            time.sleep(3.0)
            self.driver.drive(90, 90)
            time.sleep(people_cnt * 2.0)
        '''
        
        if bus_stop and (stop_M[0] - stop_m[0]) * (stop_M[1] - stop_m[1]) > (208 - 164) * (100 - 56):
            if self.bus_stop_time < time.time():
                #time.sleep(2.0)
                #self.bus_stop_time = time.time() + 2.0
                pass
            if people_cnt > 0 and self.bus_ignore_time < time.time():
                self.bus_stop_time = time.time() + 2.0
            elif self.bus_stop_time > time.time():
                pass #self.bus_ignore_time = time.time() + 5.0
        
        print(bus_stop, stop_m, stop_M, people_cnt)
        if self.bus_stop_time > time.time() and self.bus_ignore_time < time.time():
            self.driver.drive(90, 90)
        else:
            self.driver.drive(90, 115)
        return

        breaker_ret, cnt = self.vehicle_breaker_detector.detect()

        print(cnt)
        if breaker_ret:
            self.driver.drive(90,90)
        else:
            self.driver.drive(90,115)
        return
        
        light_color = self.traffic_light_detector.detect()
        print(light_color)
        if light_color == 'green':
            self.driver.drive(90, 115)
        elif light_color == 'orange':
            self.driver.drive(90, 110)
        elif light_color == 'red':
            self.driver.drive(90, 90)

        return
        obs_l, obs_m, obs_r = self.obstacle_detector.get_distance()
        line_theta,left,right = self.line_detector.detect_lines()
        line_theta += 0.4
        angle = self.steer(line_theta,left,right)
        speed = self.accelerate(angle,line_theta,left,right)
        print(line_theta)
        #print(line_theta, left, right)
        print("left: {} mid: {} right: {}".format(obs_l,obs_m,obs_r))
        cnt = 0
        s = 0
        cos_theta = 0.9
        dis = self.prev_dis
        obs_l *= cos_theta
        obs_r *= cos_theta
        if obs_l != 0:
            if len(self.prev_l) + 1 >= self.MAX_SIZE:
                self.prev_l.pop(0)
            self.prev_l.append(obs_l)
            if obs_l <= 70:
                cnt +=1
            s += obs_l
        if obs_m != 0:
            if len(self.prev_m) + 1 >= self.MAX_SIZE:
                self.prev_m.pop(0)
            self.prev_m.append(obs_m)
            if obs_m <= 70:
                cnt +=2
            s += obs_m
        if obs_r != 0:
            if len(self.prev_r) + 1 >= self.MAX_SIZE:
                self.prev_r.pop(0)
            self.prev_r.append(obs_r)
            if obs_r <=70:
                cnt +=1
            s += obs_r
        if cnt >=2:
            dis = s/cnt

        if ((cnt >=3) or (obs_m != 0 and obs_m <=70 and self.average(self.prev_m) <= 75)) or (self.average(self.prev_l) <= 75 and self.average(self.prev_m) <= 75 and self.average(self.prev_r) <=75):
            for i in range(2):
                self.driver.drive(90,90)
                time.sleep(0.1)
                self.driver.drive(90,60)
                time.sleep(0.1)
            self.driver.drive(90,90)
            time.sleep(5)
        else:
            self.driver.drive(angle + 90+ 2.77, speed)

        if cnt >=2:
            self.prev_dis = dis

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
        elif -5 <= theta <= 5:
            K = 2.0
        elif -10 <= theta <= 10:
            K = 2.3
        elif -15 <= theta <= 15:
            '''
            if theta < 0:
                if theta > -10:
                    K = 1.86
                else:
                    K = 1.4
            else:
                K = 1.75
            '''
            if theta < 0:
                K = 1.5
            else:
                K = 2.0
        else: #elif abs(theta) < 30:
            if theta < 0:
                K = 3.0
            else:
                K = 3.0
        #else:
        #    K = 
        '''
        elif theta >= 20:
            K = 3.0
        elif theta <= -20:
            K = 2.1
        else:
            K = 1.5
        '''	
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
        K = 135
        
        if abs(theta) > 4:
            self.slow_time = time.time() + 2

        if time.time() < self.slow_time:
            K = 130
        
        speed = K# - min(abs(theta)/2, 10) 

        return speed

    def exit(self):
        print('finished')

if __name__ == '__main__':
    car = AutoDrive()
    time.sleep(3)
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        car.trace()
        rate.sleep()
    rospy.on_shutdown(car.exit)

