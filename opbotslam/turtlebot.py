#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

class TurtleBot():
	def __init__(self, x, y):
		# initiliaze
        rospy.init_node('Move', anonymous=False)

		# Function on ctrl+c    
        rospy.on_shutdown(self.shutdown)
																						
		
	def new_dir(x,y):
		# Create a publisher which can "talk" to TurtleBot and tell it to move
        self.cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)

		# Twist is a datatype for velocity
        move_cmd = Twist()
		
		move_cmd.linear.x = x
		move_cmd.angular.z = y
		self.cmd_vel.publish(move_cmd)
		
	def stop():
		
		new_dir(0,0)

	def shutdown(self):
		# stop turtlebot
			rospy.loginfo("Stop TurtleBot")
		# a default Twist has linear.x of 0 and angular.z of 0.  So it'll stop TurtleBot
			self.cmd_vel.publish(Twist())
		# sleep just makes sure TurtleBot receives the stop command prior to shutting down the script
			rospy.sleep(1)

if __name__ == '__main__':
    try:
        Move()
    except:
        rospy.loginfo("Move node terminated.")