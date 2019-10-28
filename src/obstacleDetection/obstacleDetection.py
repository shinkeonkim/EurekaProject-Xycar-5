#!/usr/bin/python

import rospy, time
from std_msgs.msg import Int32MultiArray

# global variable
motor_pub = None
usonic_data = None

# init_node
def init_node():
    global motor_pub
    rospy.init_node('sample')
    rospy.Subscriber('ultrasonic', Int32MultiArray, callback)
    motor_pub = rospy.Publisher('xycar_motor_msg', Int32MultiArray, queue_size=1)

# exit_node
def exit_node():
    print('finished')

# drive
def drive(angle, speed):
    global motor_pub
    drive_info = [angle, speed]
    pub_data = Int32MultiArray(data=drive_info)
    motor_pub.publish(pub_data)

# callback
def callback(data):
    global usonic_data
    usonic_data = data.data

# sliding window moving average
def swma(vals):
    return sum(vals) / len(vals)

# exponential moving average
def ema(avg, vals):
    if len(avg) == 0:
        avg.append(vals[-1])
    weight = 0.9
    avg[0] = avg[0] * (1.0 - weight) + vals[-1] * weight
    return avg[0]
    
# median
def med(vals):
    return sorted(vals)[len(vals) // 2]
	
# main
if __name__ == '__main__':
    init_node()
    time.sleep(3)

    rate = rospy.Rate(10)
    forward = True
    
    front = []
    back = []
    
    #front_ema = []
    #back_ema = []

    while not rospy.is_shutdown():
        
        if len(front) > 4:
            front.pop(0)
        if len(back) > 4:
            back.pop(0)
        front.append(usonic_data[1])
        back.append(usonic_data[4])
	
        if forward:
            #if ema(front_ema, front) <= 40:
            #if med(front) <= 40:
            if swma(front) <= 40:
                drive(90,90)
                time.sleep(5)
                for stop_cnt in range(2):
                    drive(90, 90)
                    time.sleep(0.1)
                    drive(90, 70)
                    time.sleep(0.1)
                forward = False
            else:
                drive(90, 110)
        else:
            #if ema(back_ema, back) <= 40:
            #if med(back) <= 40:
            if swma(back) <= 40:
                drive(90,90)
                time.sleep(5)
                for stop_cnt in range(2):
                    drive(90, 90)
                    time.sleep(0.1)
                    drive(90, 110)
                    time.sleep(0.1)
                forward = True
            else:
	            drive(90, 70)
        rate.sleep()
    rospy.on_shutdown(exit_node)
