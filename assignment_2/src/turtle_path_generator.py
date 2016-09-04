#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

def circle_walker():
    pub = rospy.Publisher('turtle1/cmd_vel', Twist, queue_size=10)
    rospy.init_node('cmd_publisher_node')
    rate = rospy.Rate(10) # 10hz

    cmd = Twist()

    while not rospy.is_shutdown():
        cmd.linear.x = 1
        cmd.angular.z = 0.5

        pub.publish(cmd)
        rate.sleep()

if __name__ == '__main__':
    try:
        circle_walker()
    except rospy.ROSInterruptException:
        pass
