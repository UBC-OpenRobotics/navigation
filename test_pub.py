#!/usr/bin/env python
# license removed for brevity
import rospy
import json
from std_msgs.msg import String

def talker():
    pub = rospy.Publisher('testFollow', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    a = 1
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        a = -a
        diclist = [
            {
            "ID": 0,
            "Name": "Tim",
            "Depth": 8,
            "Angle": a,
            },
            {
            "ID": 1,
            "Name": "Roy",
            "Depth": 8,
            "Angle": -a,
            },
        ]
        list_string = json.dumps(diclist)
        rospy.loginfo(list_string)
        pub.publish(list_string)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass