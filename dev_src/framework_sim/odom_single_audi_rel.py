#! /usr/bin/env python


import math
from math import sin, cos, pi
import rospy
import tf
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, TwistStamped, Vector3


rospy.init_node('odom_rel_pub')

odom_pub = rospy.Publisher('/audi_rel_odom', Odometry, queue_size=10)
odom_broadcaster = tf.TransformBroadcaster()

x = 0.0
y = 0.0
yaw = 0.0

vx = 0.0
vy = 0.0
yawr = 0.0

odom_quat = tf.transformations.quaternion_from_euler(0, 0, yaw)

pre_time = rospy.Time.now()

init_msg = Odometry()
init_msg.header.stamp = pre_time
init_msg.header.frame_id = '/odom'

init_msg.pose.pose = Pose(Point(x, y, 0.), Quaternion(*odom_quat))

init_msg.child_frame_id = 'base_link'
init_msg.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, yawr))

rr = rospy.Rate(1.0)


def twist_callback(msg):
    global x, y, yaw, vx, vy, yawr, pre_time
    cur_time = msg.header.stamp
    vx = msg.twist.linear.x
    vy = msg.twist.linear.y
    yawr = msg.twist.angular.z

    dt = (cur_time - pre_time).to_sec()
    delta_x = (vx * cos(yaw) - vy * sin(yaw)) * dt
    delta_y = (vx * sin(yaw) + vy * cos(yaw)) * dt
    delta_yaw = yawr * dt

    x += delta_x
    y += delta_y
    yaw += delta_yaw

    odom_quat = tf.transformations.quaternion_from_euler(0, 0, yaw)

    odom_broadcaster.sendTransform(
        (x, y, 0.),
        odom_quat,
        cur_time,
        "base_link",
        "odom"
    )

    odom = Odometry()
    odom.header.stamp = cur_time
    odom.header.frame_id = '/odom'

    odom.pose.pose = Pose(Point(x, y, 0.), Quaternion(*odom_quat))

    odom.child_frame_id = 'base_link'
    odom.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, yawr))

    odom_pub.publish(odom)

    pre_time = cur_time



try:
    rospy.Subscriber('/twist', TwistStamped, twist_callback)
    odom_pub.publish(init_msg)
    rospy.spin()
except rospy.ROSInterruptException:
    print "NO"
