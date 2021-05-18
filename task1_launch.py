from speech_to_text import STT

import pyttsx3
import random
import time
import nltk
import rospy



#TODO: Robot detects bag using YOLO
#TODO: Robot navigates to bag
#TODO: Robot extends hand near bag and says "Hand me the bag"
#TODO: Robot indicates to the operator when it is ready to follow.


#TODO: Return inside arena, SLAM navigation with map localization
#TODO: 

class task1_launch():
    def __init__(self):
        #initialize
        rospy.init_node('task1', anonymous=True)
		    self.pub = rospy.Publisher('tbot/state', String, queue_size=10)

    def locate_bag(data):
        #
        pass

    def follow_person_callback(data):
        
        #TODO: "Hand me bag", then robot extends arm
        self.Subscriber("cmd_vel_mux/input/navi", Twist, queue_size=10)
        
        rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
        #TODO: Robot indicates to the operator when it is ready to follow.
        if (data.data == "True"):
            text_to_speech("I am ready to follow you")
            state_list = {"state" = "follow"}
            self.pub.publish(state_list)
            rospy.Subscriber('follow', String, follow_object_callback)
            rospy.spin()

        #TODO: Robot follows the operator towards car avoiding obstacles avoiding obstacles.


    

    def task1():
        #TODO: Operator points to bag, spin around to find it.
        text_to_speech("I am looking for the bag")
        #search for bag
        rospy.spin() #spin to find the bag
        #once found, do the following
        text_to_speech("I am pointing to the bag")
        rospy.




