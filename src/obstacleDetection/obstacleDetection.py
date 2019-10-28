# import part
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
    while not rospy.is_shutdown():
        if usonic_data[1] <= 60:
            drive(90, 90)
        else:
            drive(90, 120)
        rate.sleep()
    rospy.on_shutdown(exit_node)