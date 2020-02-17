#! /usr/bin/python


import rospy
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelState


init_pos = ModelState()
init_pos.model_name = 'audibot'
init_pos.pose.position.x = 0.0
init_pos.pose.position.y = 0.0
init_pos.pose.position.z = 0.0
init_pos.pose.orientation.x = 0.0
init_pos.pose.orientation.y = 0.0
init_pos.pose.orientation.z = 0.0
init_pos.pose.orientation.w = 0.0
init_pos.twist.linear.x = 0.0
init_pos.twist.linear.y = 0.0
init_pos.twist.linear.z = 0.0
init_pos.twist.angular.x = 0.0
init_pos.twist.angular.y = 0.0
init_pos.twist.angular.z = 0.0
thrtl_topic = '/throttle_cmd'
brake_topic = '/brake_cmd'
steer_topic = '/steering_cmd'
odom_topic = '/audi_odom'
thrtl_pub = rospy.Publisher(thrtl_topic, Float64, queue_size=10)
brake_pub = rospy.Publisher(brake_topic, Float64, queue_size=10)
steer_pub = rospy.Publisher(steer_topic, Float64, queue_size=10)
model_state_pub = rospy.Publisher('gazebo/set_model_state', ModelState, queue_size = 10, latch = True)


def odom_callback(odom):
	pos = odom.pose.pose.position.x
	if pos >= 80.0:
		thrtl_pub.publish(Float64(0.0))
		brake_pub.publish(Float64(1.0))
		model_state_pub.publish(init_pos)
	else:
		thrtl_pub.publish(Float64(1.0))
		brake_pub.publish(Float64(0.0))


def main():
	try:
		rospy.init_node('dyn_line_veh', anonymous=True)
		rospy.Subscriber(odom_topic, Odometry, odom_callback)
		thrtl_pub.publish(Float64(0.0))
		brake_pub.publish(Float64(0.0))
		steer_pub.publish(Float64(0.0))
		rospy.spin()
	except rospy.ROSInterruptException:
		print "NO"


if __name__ == '__main__':
	main()