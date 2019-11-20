import rospy
from std_msgs.msg import Int32MultiArray

class ObstacleDetector:

    def __init__(self, topic):
        self.left = -1
        self.mid = -1
        self.right = -1
        rospy.Subscriber(topic, Int32MultiArray, self.read_distance)

    def read_distance(self, data):
        self.left = data.data[0]
        self.mid = data.data[1]
        self.right = data.data[2]

    def get_distance(self):
        return self.left, self.mid, self.right
