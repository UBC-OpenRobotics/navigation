#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

class TurtleBot():
	DEPTH = 5
	def __init__(self):
		# initiliaze
		self.error = 0
		self.follow_id = 0
		rospy.init_node('Move', anonymous=False)

	def new_dir(self, x,y):
		# Create a publisher which can "talk" to TurtleBot and tell it to move
		self.cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)

		# Twist is a datatype for velocity
		move_cmd = Twist()

		move_cmd.linear.x = x
		move_cmd.angular.z = y
		self.cmd_vel.publish(move_cmd)

	def stop(self):
		self.new_dir(0,0)
	
	def follow(self, plist):
		# list of a person being followed - ID, Name, Depth, Right/Left
		# Complete right is 1 and complete left is -1
		follow_id = self.follow_id
		res = next((item for item in plist if item['ID'] == follow_id), None)
		if res:
			error_prev = self.error
			error = res['Depth'] - DEPTH
			p = 0.2 * error
			linearX = p
			if res['Angle'] == 1:
				AngularY = 0.2
			elif res['Angle'] == -1:
				AngularY = -0.2
			else:
				AngularY = 0
			new_dir(linearX,AngularY)
			self.error = error

	def follow_id(self, id_number):
		self.follow_id = id_number

	def shutdown(self):
		# stop turtlebot
			rospy.loginfo("Stop TurtleBot")
		# a default Twist has linear.x of 0 and angular.z of 0.  So it'll stop TurtleBot
			self.cmd_vel.publish(Twist())
		# sleep just makes sure TurtleBot receives the stop command prior to shutting down the script
			rospy.sleep(1)

if __name__ == '__main__':
	try:

		tbot = TurtleBot()
		# Function on ctrl+c
		rospy.on_shutdown(tbot.shutdown)
	except:
		rospy.loginfo("Move node terminated.")
