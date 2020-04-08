#!/usr/bin/env python

import rospy
from route_publisher import loadRoute
from route_publisher.msg import Route

if __name__ == '__main__':
    # Load and publish the route
    rospy.init_node('route_publisher', anonymous=True)
    route_filename = rospy.get_param('~route_filename')
    route = loadRoute(route_filename)
    rospy.Publisher('/route', Route, queue_size=1, latch=True).publish(route)

    print('route pub running!')
    rospy.spin()
