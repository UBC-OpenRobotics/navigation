#!/usr/bin/env python
import tf
import geometry_msgs
import rospy
from geometry_msgs.msg import Twist, Pose, Point, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *
import math

class TurtleBot():
	DEPTH = 5
	MODES = ('resting', 'follow', 'map_navigating')

	def __init__(self):
		# initiliaze
		self.error_depth = 0
		self.id = 0
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
		if self.mode != 'follow':
			return
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

	def navigate(self, pos, quat):
    	if self.mode != 'navigate':
    			return
		self.goal_sent = True
		goal = MoveBaseGoal()
		goal.target_pose.header.frame_id = 'map'
		goal.target_pose.header.stamp = rospy.Time.now()
		goal.target_pose.pose = Pose(Point(pos['x'], pos['y'], 0.000),
								Quaternion(quat['r1'], quat['r2'], quat['r3'], quat['r4']))

		# Start moving
		self.move_base.send_goal(goal)

		# Allow TurtleBot up to 60 seconds to complete task
		success = self.move_base.wait_for_result(rospy.Duration(60)) 

		state = self.move_base.get_state()
		result = False

		if success and state == GoalStatus.SUCCEEDED:
			# We made it!
			result = True
		else:
			self.move_base.cancel_goal()

		self.goal_sent = False
		return result

	def shutdown(self):
		# stop turtlebot
			rospy.loginfo("Stopped TurtleBot")
		# a default Twist has linear.x of 0 and angular.z of 0.  So it'll stop TurtleBot
			self.cmd_vel.publish(Twist())
		# sleep just makes sure TurtleBot receives the stop command prior to shutting down the script
			rospy.sleep(1)

def callback(data):
	rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
	loaded_dictionary = json.loads(data.data)
	if loaded_dictionary['state'] == "follow":
		tbot.follow(loaded_dictionary['details'])
	if loaded_dictionary['state'] == "navigate":
    		tbot.follow(loaded_dictionary['details'])
	# getattr(tbot, loaded_dictionary['state'])()

def set_mode(data):
	"""data: str"""
	tbot.mode = data.data

if __name__ == '__main__':
	try:
		tbot = TurtleBot()
		# Function on ctrl+c
		rospy.on_shutdown(tbot.shutdown)
		rospy.Subscriber("tbot/state", String, set_mode)
		# spin() simply keeps python from exiting until this node is stopped
		rospy.spin()
	except:
		rospy.loginfo("Move node terminated.")
