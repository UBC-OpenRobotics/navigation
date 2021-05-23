from speech_to_text import STT
from std_msgs.msg import String
from geometry_msgs.msg import Twist

import pyttsx3
import random
import time
import nltk
import rospy

#TODO: Robot detects bag using YOLO
#TODO: Robot navigates to bag
#TODO: Robot extends hand near bag and says "Hand me the bag"
#TODO: Robot indicates to the operator when it is ready to follow.

class task1_launch():
    def __init__(self):
        #initialize
        self.state_pub = rospy.Publisher('tbot/state', String, queue_size=10)
        self.target_pub  = rospy.Publisher('/tbot/target', String, queue_size=10)   
        self.navigate_pub = rospy.Publisher('map_navigate', String, queue_size=10)
        self.carry_pub = rospy.Publisher('arm_pose', String, queue_size=10)
        rospy.init_node('task1', anonymous=True) 

    def locate_bag(self, data):
        #Define robot's target object
        ##self.Subscriber("cmd_vel_mux/input/navi", Twist, queue_size=10)
        target = "bag"
        self.target_pub.publish(target)

        #Have robot follow path to bag
        state_list = {"state" = "follow"}
        self.state_pub.publish(state_list)

        #TODO: "Hand me bag", then robot extends arm
        text_to_speech("Please hand me the bag")
        self.carry_pub.publish("carry") #confirm this implementation with Andrew

    def follow_object_callback(data):
         rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
        if (data.data['depth'] == 0 and data.data['angle'] == 0)

    def follow_person_callback(self, data):
        rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
        #TODO: Robot indicates to the operator when it is ready to follow.
        if (data.data == "True"):
            text_to_speech("I am ready to follow you")
            state_list = {"state" = "follow"}
            self.state_pub.publish(state_list)
            rospy.Subscriber('follow', String, follow_object_callback)
            rospy.spin()

    #TODO: Robot follows the operator towards car avoiding obstacles avoiding obstacles.    
    def follow_person(self, data):
        state_list = {"state" = "follow"}
        self.pub.publish(state_list)
        rospy.Subscriber('follow', String, follow_person_callback)
        rospy.spin()

    #TODO: Return inside arena, SLAM navigation with map localization
    #TODO: Robot returns to starting point
    # mock current orientation = 0,0,0,1
    def return_inside(self, data):
        if self.mode != 'map_navigating':
            return
		nav_list = json.loads(data.data)
		result = "False"
		print(nav_list)
		x_current = nav_list['current']['x']
		y_current = nav_list['current']['y']
		x_target = nav_list['target']['x']
		y_target = nav_list['target']['y']
		quaternion = (nav_list['current']['r1'], nav_list['current']['r2'], nav_list['current']['r3'], nav_list['current']['r4'])
		euler = tf.transformations.euler_from_quaternion(quaternion)
		list_current_orientation = euler[2]
		target_angle = math.atan((y_target - y_current) / (x_target - x_current))
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
		
		rospy.loginfo("Navigated to target")
		result = "True"
		navigate_publish = rospy.Publisher('navigate', String, queue_size=10)
		navigate_publish.publish(result)


    def task1():
        while not rospy.is_shutdown():
            #TODO: Operator points to bag, spin around to find it.
            text_to_speech("I am looking for the bag")
            locate_bag()

            #search for bag
            rospy.spin() #spin to find the bag
            #once found, do the following
            text_to_speech("I am pointing to the bag")

if __name__ == '__main__':
    try:
        launch = task1_launch()
    except rospy.ROSInterruptException:
        pass



