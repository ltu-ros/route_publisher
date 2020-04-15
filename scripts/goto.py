#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist, Point, PointStamped
from nav_msgs.msg import Odometry
from math import pow, atan2, sqrt, pi
from visualization_msgs.msg import Marker
import tf


class GotoNode:

    def __init__(self):
        rospy.init_node('goto_node', anonymous=True)

        twist_topic = rospy.get_param('~twist_topic', '/prizm/twist_controller/twist_cmd')
        goal_topic  = rospy.get_param('~goal_topic',  '/goto_node/goal')
        odom_topic  = rospy.get_param('~odom_topic',  '/car/odom')


        self.twist_pub  = rospy.Publisher(twist_topic, Twist, queue_size=10)
        self.marker_pub = rospy.Publisher(goal_topic, Marker, queue_size=10, latch=True)
        self.odom_sub   = rospy.Subscriber(odom_topic, Odometry, self.odomCallback)
        self.goal_sub   = rospy.Subscriber('/clicked_point', PointStamped, self.clickedPointCallback)

        self.goal = None


    def clickedPointCallback(self, msg):
        self.goalCallback(msg.point)


    def goalCallback(self, msg):
        print('Got new goal: {}, {}'.format(msg.x, msg.y))
        self.goal = msg
        self.pubGoalMarker()


    def pubGoalMarker(self):
        if self.goal is None: return
        m = Marker()
        m.id = 1
        m.header.frame_id = 'map'
        m.type = Marker.SPHERE
        m.pose.position = self.goal
        m.pose.orientation.w = 1.0
        m.color.a = 1
        m.color.r = 1
        m.scale.x, m.scale.y, m.scale.z = (0.2, 0.2, 0.2)
        self.marker_pub.publish(m)


    def odomCallback(self, msg):
        if self.goal is None: return
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        quat = msg.pose.pose.orientation
        q = [quat.x, quat.y, quat.z, quat.w]
        _, _, yaw = tf.transformations.euler_from_quaternion(q)
        dist = sqrt(pow(self.goal.x - x, 2) + pow(self.goal.y - y, 2))

        steering_angle = atan2(self.goal.y - y, self.goal.x - x)
        z = steering_angle - yaw
        z0 = z
        # Rollovers from 0 to 2pi
        if z < -pi:
            z = -1 * (z + pi)
        elif z > pi:
            z = -1 * (z - pi)

        # print('D={}, yaw={}, steer={}, z0={}, z={}'.format(dist, yaw, steering_angle, z0, z))

        out = Twist()

        if dist < 0.01:
            # Reached goal, stop
            out.linear.x = 0
            out.angular.z = 0
            self.goal = None
        else:
            out.linear.x = min(0.5, dist ** 3)
            out.angular.z = max(-1.0, min(1.0, z*2))

        self.twist_pub.publish(out)


if __name__ == '__main__':
    g = GotoNode()
    print('goto running!')
    rospy.spin()
