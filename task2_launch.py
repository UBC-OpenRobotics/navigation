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

class task2_launch():
    def __init__(self):
    	# initiliaze
        rospy.init_node('task2', anonymous=True)
        self.name = "name"
        self.location = "location"
        self.pub = rospy.Publisher('tbot/state', String, queue_size=10)
        self.navigate_pub = rospy.Publisher('map_navigate', String, queue_size=10)


    def nav_callback(data):
        rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
        if (data.data == "True"):
            text_to_speech(f'Hi {self.name} has arrived. They are at {self.location}')
            time.sleep(3)
            self.task2_loop()

    def follow_callback(data):
        rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
        if (data.data['depth'] == 0 &&  data.data['angle'] == 0)
            text_to_speech("Welcome to the event. Could I please get your name?")
            response = stt.recognize()
            print(response)
            name, nameFound = ext.extract_entity(response, is_name = True) 
            while(not nameFound):
                text_to_speech("Sorry, I didn't understand. Could I please get your name again?")
                response = stt.recognize()
                print(response)
                name, nameFound = ext.extract_entity(response, is_name = True)
                self.name = name

            text_to_speech(f'Hi {name}. Welcome to the party! I will let the operator know of your arrival.')
            
            # TODO: Get person description - francisco implemented a node that spits out all the description apparently
            # TODO: Get person location
            state_list = {"state" = "map_navigating"}
            nav_list = {"current": {"x": 0, "y": 0, "r1": 0, "r2": 0, "r3": 0, "r4": 1},
                        "target": {"x": 2, "y:" 10}}
            self.pub.publish(state_list)
            self.navigate_pub.publish(nav_list)
            rospy.Subscriber('navigate', String, nav_callback)
            rospy.wait_for_message()


    def task2():  
        # Setting wake word to Robot
        text_to_speech("Hello. If you would like my assistance, just say Hey Robot and I will gladly help you!")
        response = stt.wake_word("Robot")

        # print("Initiates guest recognition process")
        time.sleep(3)
        self.task2_loop()

    def tas2_loop():
        #TODO: Andrew, how should we change follow id here?
        state_list = {"state" = "follow"}
        self.pub.publish(state_list)
        rospy.Subscriber('follow', String, follow_callback)
        rospy.wait_for_message()

if __name__ == '__main__':
    try:
        launch = task2_launch()
    except rospy.ROSInterruptException:
        pass