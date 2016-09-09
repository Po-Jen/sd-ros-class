#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

ranges = []

def callback(data):
    global ranges
    ranges = data.ranges
    # (data.angle_max - data.angle_min)/data.angle_increment: 638.99
    # len(data.ranges):640

def random_walker():
    pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)
    sub = rospy.Subscriber('/scan', LaserScan, callback)
    rospy.init_node('turtlebot_random_exploration_node')
    rate = rospy.Rate(10)

    cmd = Twist()

    while not rospy.is_shutdown():
        #data.range_min == 0, can be ignored in this case
        global ranges
        distance_smaller_than_thres = [i for i in ranges if i <= 0.5]

        if( len(distance_smaller_than_thres) > 0):
            cmd.linear.x = 0
            cmd.angular.z = 0.5
            
            pub.publish(cmd)
            rate = rospy.Rate(1)
            rate.sleep()
        else:
            cmd.linear.x = 0.1
            cmd.angular.z = 0
            pub.publish(cmd)
            rate = rospy.Rate(10)
            rate.sleep()


if __name__ == '__main__':
    try:
        random_walker()
    except rospy.ROSInterruptException:
        pass

