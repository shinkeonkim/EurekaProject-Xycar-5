import rospy
from std_msgs.msg import Int32MultiArray

class MotorDriver:

    def __init__(self, topic):
        self.motor_pub = rospy.Publisher(topic,
                                         Int32MultiArray,
                                         queue_size=1)

    def drive(self, angle, speed):
        drive_info = [angle, speed]
        pub_info = Int32MultiArray(data=drive_info)
        self.motor_pub.publish(pub_info)
