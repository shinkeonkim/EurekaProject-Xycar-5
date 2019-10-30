#!/usr/bin/python

import rospy, time
from std_msgs.msg import Int32MultiArray

# global variable
motor_pub = None
usonic_data = None

front = []
back = []

front_ema = []
back_ema = []

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
    global front
    global back

    # front, back is nonzero, take a data.
    if data.data[1] != 0 and data.data[4] != 0 :
        usonic_data = data.data
        
	if len(front) > 4:
            front.pop(0)
        if len(back) > 4:
            back.pop(0)

        front.append(usonic_data[1])
        back.append(usonic_data[4])
    
# sliding window moving average
def swma(vals):
    return sum(vals) / len(vals)

# exponential moving average
def ema(avg, vals):
    if len(avg) == 0:
        avg.append(vals[-1])
    weight = 0.9
    avg[0] = avg[0] * weight + vals[-1] * (1.0 - weight)
    return avg[0]
    
# median
def med(vals):
    return sorted(vals)[len(vals) // 2]

# main
if __name__ == '__main__':
    init_node()
    time.sleep(3)

    
    rate = rospy.Rate(100)
    forward = True
    
    forward_cnt = 0
    forward_speed = [110,115,120]
    forward_std_value = [48,65,85]

    backward_speed = 70
    back_std_value = 54

    # add usonic_data before forward
    '''
    for i in range(5):
        front.append(usonic_data[1])
        back.append(usonic_data[4])
        rate.sleep()
    '''

    while not rospy.is_shutdown():
        print(front,back,usonic_data)
        if forward:
            #if ema(front_ema, front) <= forward_std_value[forward_cnt]:
            if med(front) <= forward_std_value[forward_cnt]:
            #if swma(front) <= forward_std_value[forward_cnt]:
                for stop_cnt in range(2):
                    drive(90, 90)
                    time.sleep(0.1)
                    drive(90, backward_speed)
                    time.sleep(0.1)
                drive(90,90)
                time.sleep(5)
                forward = False        
                forward_cnt+=1
                forward_cnt%=3
            else:
                drive(90, forward_speed[forward_cnt])
        else:
            #if ema(back_ema, back) <= back_std_value:
            if med(back) <= back_std_value:
            #if swma(back) <= back_std_value:
                for stop_cnt in range(2):
                    drive(90, 90)
                    time.sleep(0.1)
                    drive(90, 110)
                    time.sleep(0.1)
                drive(90,90)
                time.sleep(5)
                forward = True
            else:
	        drive(90, backward_speed)
        rate.sleep()

    rospy.on_shutdown(exit_node)
