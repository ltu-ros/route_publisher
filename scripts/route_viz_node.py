#!/usr/bin/env python

import rospy
from route_publisher.msg import Route
from visualization_msgs.msg import Marker, MarkerArray

def colorhash(x):
    h = hash(str(x))
    return ((h%155+100)/255.0,
            ((h//100)%155+100)/255.0,
            ((h//10000)%155+100)/255.0)

def makeMarker(x, y, id, color, z=0):
    scale = 0.3
    m = Marker()
    m.id = id
    m.header.frame_id = 'map'
    m.type = Marker.SPHERE
    m.color.r, m.color.g, m.color.b = color
    m.color.a = 1.0
    m.pose.position.x = x
    m.pose.position.y = y
    m.pose.position.z = z
    m.scale.x = scale
    m.scale.y = scale
    m.scale.z = scale
    m.pose.orientation.w = 1.0
    return m


def makeLabel(x, y, id, text, z=0):
    m = Marker()
    m.id = id
    m.header.frame_id = 'map'
    m.type = Marker.TEXT_VIEW_FACING
    m.text = text
    m.color.r, m.color.g, m.color.b = (1,1,1)
    m.color.a = 1.0
    m.pose.position.x = x
    m.pose.position.y = y
    m.pose.position.z = z + 1 # +1 to be above std marker
    m.pose.orientation.w = 1.0
    m.scale.z = 1.0
    return m


def routeCallback(route):
    print('viz got route message!')
    count = 0
    markers = []
    last_x = 0
    last_y = 0
    for command in route.commands:
        color = colorhash(command.command)

        # Get X/Y
        x, y = command.x, command.y
        if command.command in ('wait', 'search'):
            x, y = last_x, last_y

        # Point
        markers.append(makeMarker(x, y, count, color))
        count += 1

        # Label
        markers.append(makeLabel(x, y, count, command.name))
        count += 1

        last_x = x
        last_y = y


    global publisher
    publisher.publish(MarkerArray(markers=markers))



rospy.init_node('route_viz', anonymous=True)
publisher = rospy.Publisher('/route_viz', MarkerArray, queue_size=10, latch=True)
rospy.Subscriber('/route', Route, routeCallback)

print('route viz running!')
rospy.spin()
