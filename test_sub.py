#!/usr/bin/env python
import rospy
import json
from std_msgs.msg import String
import Turtlebot

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    loaded_dictionary = json.loads(data.data)
    turtlebot.follow(loaded_dictionary)
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)
    turtlebot.follow_id(0)
    rospy.Subscriber("testFollow", String, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    turtlebot = Turtlebot()
    listener()
    