#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

flag = 0;

def callback(data):
    print "angle_min: " + str(data.angle_min)
    print "angle_max: " + str(data.angle_max)
    global flag
    flag = flag + 1;

def random_walker():
    pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)
    sub = rospy.Subscriber('/scan', LaserScan, callback)
    rospy.init_node('turtlebot_random_exploration_node')
    rate = rospy.Rate(10)

    cmd = Twist()

    while not rospy.is_shutdown():
        cmd.linear.x = -0.1

        pub.publish(cmd)
        print flag
        rate.sleep()


if __name__ == '__main__':
    try:
        random_walker()
    except rospy.ROSInterruptException:
        pass

