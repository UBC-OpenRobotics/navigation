#!/usr/bin/env python
import tf
import geometry_msgs
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Pose, Point, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *
import math
import json

class TurtleBot():
	DEPTH = 5
	MODES = ('resting', 'follow', 'map_navigating')

	def __init__(self):
		# initiliaze
		self.error_depth = 0
  		self.error_angle = 0
		self.id = 'person 1'
  		# Create a publisher which can "talk" to TurtleBot and tell it to move
		self.cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)
  
		self.error_depth_angle = 0
		self.goal_sent = False
		self._mode = "resting"
		rospy.init_node('Turtlebot_wrapper', anonymous=False)
		# Tell the action client that we want to spin a thread by default
		self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
		rospy.loginfo("Wait for the action server to come up")

		# Allow up to 5 seconds for the action server to come up
		self.move_base.wait_for_server(rospy.Duration(5))

	@property
	def mode(self):
		print("Turtlebot mode: " + self._mode)
		return self._mode

	@mode.setter
	def mode(self, mode):
		print("Turtlebot mode: " + self._mode)
		if not mode in self.MODES:
			raise ValueError("Invalid Turtlebot Mode")
		self._mode = mode

	def new_dir(self, x,y):
		# Twist is a datatype for velocity
		move_cmd = Twist()
		move_cmd.linear.x = x
		move_cmd.angular.z = y
		self.cmd_vel.publish(move_cmd)

	def stop(self):
		self.new_dir(0,0)
	
	def follow(self, data):
		if self.mode != 'follow':
			return
		plist = json.loads(data.data)
		if not plist:
			self.stop
   			return
  		print(plist)
		# list of a person being followed - ID, Name, Depth, Right/Left
		# Complete right is 1 and complete left is -1
		follow_id = self.id
		res = next((item for item in plist if item['id'] == follow_id), None)
		if res:
			error_depth_prev = self.error_depth
			error_angle_prev = self.error_angle
			error_depth = res['depth'] - self.DEPTH
			error_angle = res['angle'] - 0
			p = 0.2 * error_depth
			p_angle = 0.3 * error_angle
			linearX = p
			if res['angle'] != 0:
				AngularY = -p_angle
			else:
				AngularY = 0
			print(AngularY)
			self.new_dir(linearX,AngularY)
			self.error_depth = error_depth
			self.error_angle = error_angle

			follow_list = {"depth" = linearX, "angle" = AngularY}
			follow_pub = rospy.Publisher('follow', String, queue_size = 10)
			follow_pub.pub(follow_list)

	def follow_id(self, id_number):
		self.id = id_number

	# mock current orientation = 0,0,0,1
	# mock current distance = 0,0
	# mock target kitchen = 20,-4
	# mock target living room = 2, 10
	def navigate(self, list_current_distance, list_current_orientation, list_target):
		if self.mode != 'map_navigating':
        		return
		x_current = list_current_distance['x']
		y_current = list_current_distance['y']
		x_target = list_target['x']
		y_target = list_target['y']
		euler = tf.transformations.euler_from_quaternion(list_current_orientation)
		list_current_orientation = euler[2]
		target_angle = math.atan((y_target - y_current) / (x_target - x_cuurent))
		angle = 0.2
		while(angle != 0):
			angle = 0.2 * (target_angle - list_current_orientation)
			self.new_dir(0, angle)
		pos = 0.2
		distance = sqrt(pow((x_target - x_current),2) - pow((y_target - y_current),2))
		while(pos != 0):
			pos = 0.1 * distance
			distance -= 0.1
			self.new_dir(pos, 0)

	def shutdown(self):
		# stop turtlebot
			rospy.loginfo("Stopped TurtleBot")
		# a default Twist has linear.x of 0 and angular.z of 0.  So it'll stop TurtleBot
			self.stop()
		# sleep just makes sure TurtleBot receives the stop command prior to shutting down the script
			rospy.sleep(1)

# def callback(data):
# 	rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
# 	if loaded_dictionary['state'] == "follow":
# 		tbot.follow(loaded_dictionary['details'])
	# getattr(tbot, loaded_dictionary['state'])()

def set_mode(data):
	"""data: str"""
	rospy.loginfo(rospy.get_caller_id() + " I heard %s", data.data)
	tbot.mode = data.data
	print(tbot.mode)

if __name__ == '__main__':
 	try:
		tbot = TurtleBot()
		print(rospy.is_shutdown())
		# Function on ctrl+c
		rospy.on_shutdown(tbot.shutdown)

		rospy.Subscriber("tbot/state", String, set_mode)
		# TODO: use other  message types other than JSON String for the following subscribers
		rospy.Subscriber("ppl_detected", String, tbot.follow) 
		# spin() simply keeps python from exiting until this node is stopped
		rospy.spin()
	except:
		rospy.loginfo("Move node terminated.")
