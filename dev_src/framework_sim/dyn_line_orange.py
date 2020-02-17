#! /usr/bin/python


import rospy
import numpy as np
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelState


node_name = 'dyn_line_orange'
thrtl_topic = '/orange/throttle_cmd'
brake_topic = '/orange/brake_cmd'
steer_topic = '/orange/steering_cmd'
odom_topic = '/orange/odom'
modelstate_topic = 'gazebo/set_model_state'
model_name = 'orange'
init_pos = ModelState()
init_pos.model_name = model_name
roll = 0.0
pitch = 0.0
yaw = 1.57
qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
init_pos.pose.position.x = 40.0
init_pos.pose.position.y = -45.0
init_pos.pose.position.z = 0.0
init_pos.pose.orientation.x = qx
init_pos.pose.orientation.y = qy
init_pos.pose.orientation.z = qz
init_pos.pose.orientation.w = qw
init_pos.twist.linear.x = 0.0
init_pos.twist.linear.y = 0.0
init_pos.twist.linear.z = 0.0
init_pos.twist.angular.x = 0.0
init_pos.twist.angular.y = 0.0
init_pos.twist.angular.z = 0.0
thrtl_pub = rospy.Publisher(thrtl_topic, Float64, queue_size=10)
brake_pub = rospy.Publisher(brake_topic, Float64, queue_size=10)
steer_pub = rospy.Publisher(steer_topic, Float64, queue_size=10)
model_state_pub = rospy.Publisher(modelstate_topic, ModelState, queue_size = 10, latch = True)


def odom_callback(odom):
    ref = 50.0
    tol = ref / 2.0
    pos = odom.pose.pose.position.y
    if pos >= ref:
        thrtl_pub.publish(Float64(0.0))
        brake_pub.publish(Float64(1.0))
        model_state_pub.publish(init_pos)
    elif pos >= tol:
        thrtl_pub.publish(Float64(0.0))
        brake_pub.publish(Float64(((pos - tol) / (ref - tol))))
    elif pos == 0.0:
        thrtl_pub.publish(Float64(1.0))
        brake_pub.publish(Float64(0.0))
    else:
        thrtl_pub.publish(Float64((1.0 - (pos / tol))))
        brake_pub.publish(Float64(0.0))


def main():
    try:
        rospy.init_node(node_name, anonymous=True)
        rospy.Subscriber(odom_topic, Odometry, odom_callback)
        thrtl_pub.publish(Float64(0.0))
        brake_pub.publish(Float64(0.0))
        steer_pub.publish(Float64(0.0))
        rospy.spin()
    except rospy.ROSInterruptException:
        print "NO"


if __name__ == '__main__':
    main()