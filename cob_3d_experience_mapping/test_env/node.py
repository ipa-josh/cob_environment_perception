#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry

from context import Context, Robot
from transition import Position, Action

class Node:
    
	def __init__():
		rospy.init_node('exp', anonymous=False)
		
		self.robot = Robot(1,1,1)
		self.ctxt = Context(self.robot)

		rospy.Subscriber("odom", Odometry, self.on_odom)
	
	def on_odom(data):
		action = Action()
		action.data = [data.twist.twist.linear.x, data.twist.twist.angular.z]
		action.time = data.header.stamp.to_sec()
		self.ctxt.on_action(action)


if __name__ == '__main__':
    n = Node()
    rospy.spin()
