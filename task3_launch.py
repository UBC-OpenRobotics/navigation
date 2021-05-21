#!/usr/bin/env python3

from speech_to_text import STT
from extractor import Extractor
from nltk.tag.stanford import StanfordNERTagger

import pyttsx3
import random
import time
import nltk
import rospy

ext = Extractor()
stt = STT()
tts = pyttsx3.init()

def text_to_speech(text):
    print(text)
    tts.say(text)
    tts.runAndWait()

class task3_launch():
    def __init__(self):
    	# initiliaze
        rospy.init_node('task3', anonymous=True)
		self.pub = rospy.Publisher('tbot/state', String, queue_size=10)
        self.navigate_pub = rospy.Publisher('map_navigate', String, queue_size=10)

    def folllow_object_callback(data):
        rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
        if (data.data['depth'] == 0 &&  data.data['angle'] == 0)
            # TODO: point to empty seat

    def follow_person_callback(data):
        rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
        if (data.data['depth'] == 0 &&  data.data['angle'] == 0)
            # TODO: Face and point guest introductions
            text_to_speech(f'Hi this is {name}. Their favorite drink is {drink}')
            # TODO: Empty seat detection
            # TODO: fix follow
            state_list = {"state" = "follow"}
            self.pub.publish(state_list)
            rospy.Subscriber('follow', String, follow_object_callback)
            rospy.spin()

    def nav_callback(data):
        rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
        if (data.data == "True")
            text_to_speech("Welcome to the event. Could I please get your name?")
            response = stt.recognize()
            print(response)
            name, nameFound = ext.extract_entity(response, is_name = True) 
            while(not nameFound):
                text_to_speech("Sorry, I didn't understand. Could I please get your name again?")
                response = stt.recognize()
                print(response)
                name, nameFound = ext.extract_entity(response, is_name = True)

            text_to_speech(f'Hi {name}. What is your favorite drink?')
            response = stt.recognize()
            print(response)
            drink, drinkFound = ext.extract_entity(response, is_drink = True)

            while(not drinkFound):
                text_to_speech("Sorry, I didn't understand. Could I please get your drink again?")
                response = stt.recognize()
                print(response)
                drink, drinkFound = ext.extract_entity(response, is_drink = True)
            # TODO: Person detection John
            # TODO: fix follow
            state_list = {"state" = "follow"}
            self.pub.publish(state_list)
            rospy.Subscriber('follow', String, follow_person_callback)
            rospy.spin()


    def task3():  
        # Setting wake word to Robot
        text_to_speech("Hello. If you would like my assistance, just say Hey Robot and I will gladly help you!")
        response = stt.wake_word("Robot")

        # print("Initiates guest recognition process")
        time.sleep(3)
        # TODO: sequence to make robot aware that a guest is arriving

        # mock current orientation = 0,0,0,1
	    # mock current distance = 0,0
	    # mock target kitchen = 20,-4
	    # mock target living room = 2, 10
        nav_list = {"current": {"x": 0, "y": 0, "r1": 0, "r2": 0, "r3": 0, "r4": 1},
                    "target": {"x": 20, "y:" -4}}
        self.navigate_pub.publish(nav_list)
        state_list = {"state" = "map_navigating"}
        self.pub.publish(state_list)
        rospy.Subscriber('navigate', String, nav_callback)
        rospy.spin()

if __name__ == '__main__':
    try:
        launch = task3_launch()
    except rospy.ROSInterruptException:
        pass