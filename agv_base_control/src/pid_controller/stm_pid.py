import rospy
from nav_msgs.msg import Odometry

def publisher():
    rospy.init_node("stm_pid_pub", anonymous=True)
    publisherName = rospy.Publisher('/pidoutput', xx, queue_size = 10)
    rate = rospy.Rate(10) #frequence
    while not rospy.is_shutdown() :
        #
        #
        #

def listener():
    rospy.init_node("stm_pid_sub", anonymous=True)
    rospy.Subscriber("odom", Odometry, callback)

def callback(msg):
    rospy.loginfo(msg.pose.pose)

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterrupException:
        pass