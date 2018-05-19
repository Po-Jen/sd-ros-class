#!/usr/bin/env python

import rospy
from rospy import ServiceException
from motion_detector.srv import Mode

node_name = 'motion_mode_keyboard'
service_name = 'keyboard'

instructions = '''
Choose a command:
r (raw video)
f (farneback optical flow)
m (MOG2)
'''

if __name__ == '__main__':
    rospy.init_node(node_name)
    rospy.wait_for_service(service_name)

    change_mode = rospy.ServiceProxy(service_name, Mode)    # client

    while not rospy.is_shutdown():  # ros::ok
        try:
            mode = raw_input(instructions)
            response = change_mode(mode)

            if not response.error:
                print 'Mode changed to: ' + mode
            else:
                raise ServiceException(response.error)
       
        except ServiceException as e:
            print(e)
