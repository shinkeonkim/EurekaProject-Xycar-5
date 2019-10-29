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

# main
if __name__ == '__main__':
    init_node()
    time.sleep(3)

    rate = rospy.Rate(10)
    forward = True
    
    forward_cnt = 0
    forward_speed = [110,120,130]
    forward_std_value = [40,40,40]

    backward_speed = 70
    back_std_value = 40

    front = []
    back = []

    while not rospy.is_shutdown():
        
        if len(front) > 4:
            front.pop(0)
        if len(back) > 4:
            back.pop(0)
        front.append(usonic_data[1])
        back.append(usonic_data[4])
        
        front_value = sum(front) / len(front)
        back_value = sum(back) / len(back)
        
        if forward:
            if front_value <= forward_std_value[forward_cnt]:
                drive(90,90) # 정지 5초
                time.sleep(5)
                for stop_cnt in range(2):
                    drive(90, 90)
                    time.sleep(0.1)
                    drive(90, backward_speed)
                    time.sleep(0.1)
                forward = False        
                forward_speed_cnt+=1
                forward_speed_cnt%=3
            else:
                drive(90, forward_speed[forward_cnt])
        else:
            if back_value <= back_std_value:
                drive(90,90) # 정지 5초
                time.sleep(5)
                for stop_cnt in range(2):
                    drive(90, 90)
                    time.sleep(0.1)
                    drive(90, forward_speed[forward_cnt])
                    time.sleep(0.1)
                forward = True
            else:
	            drive(90, backward_speed)
        rate.sleep()

    rospy.on_shutdown(exit_node)
