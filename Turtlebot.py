#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

class TurtleBot():
	DEPTH = 5
	def __init__(self):
		# initiliaze
		self.error_depth = 0
		self.id = 0
		self.error_depth_angle = 0
		rospy.init_node('Turtlebot_wrapper', anonymous=False)

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
		follow_id = self.id
		res = next((item for item in plist if item['id'] == follow_id), None)
		if res:
			error_depth_prev = self.error_depth
			error_angle_prev = self.error_angle
			error_depth = res['depth'] - DEPTH
			error_angle = res['angle'] - 0
			p = 0.2 * error_depth
			p_angle = 0.2 * error_angle
			linearX = p
			if res['angle'] > 0:
				AngularY = p_angle
			elif res['angle'] < 0:
				AngularY = -p_angle
			else:
				AngularY = 0
			self.new_dir(linearX,AngularY)
			self.error_depth = error_depth
			self.error_angle = error_angle

	def follow_id(self, id_number):
		self.id = id_number

	def shutdown(self):
		# stop turtlebot
			rospy.loginfo("Stop TurtleBot")
		# a default Twist has linear.x of 0 and angular.z of 0.  So it'll stop TurtleBot
			self.cmd_vel.publish(Twist())
		# sleep just makes sure TurtleBot receives the stop command prior to shutting down the script
			rospy.sleep(1)

def listener():

	# In ROS, nodes are uniquely named. If two nodes with the same
	# name are launched, the previous one is kicked off. The
	# anonymous=True flag means that rospy will choose a unique
	# name for our 'listener' node so that multiple listeners can
	# run simultaneously.
	rospy.Subscriber("tbot/state", String, callback)

	# spin() simply keeps python from exiting until this node is stopped
	rospy.spin()

def callback(data):
	rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
	loaded_dictionary = json.loads(data.data)
	if loaded_dictionary['state'] == "follow":
		tbot.follow(loaded_dictionary['details'])
	# getattr(tbot, loaded_dictionary['state'])()

if __name__ == '__main__':
	try:

		tbot = TurtleBot()
		# Function on ctrl+c
		rospy.on_shutdown(tbot.shutdown)
		listner()
	except:
		rospy.loginfo("Move node terminated.")
