#!/usr/bin/env python3

import rospy 
import math
from geometry_msgs.msg import Twist
from navigation.msg import wheel_encoder_data

FAKE_WHEEL_RADUIS_IN_METERS = 0.05
rpm = 0
spin_direction = True

def fake_wheel_encoder_publisher():
    rospy.init_node("fake_wheel_encoder_publisher")
    publisher_right = rospy.Publisher("fake_whheel_encoder_publisher/right", wheel_encoder_data, queue_size=5) # change queue_size if needed
    publisher_left = rospy.Publisher("fake_whheel_encoder_publisher/left", wheel_encoder_data, queue_size=5)
    rate = rospy.Rate(1) 

    is_dynamic = rospy.get_param('dynamic', False)
    if(is_dynamic):
        rospy.Subscriber("cmd_vel", Twist, vel_cb)

    while not rospy.is_shutdown():
        wheel_data = wheel_encoder_data()
        wheel_data.rpm = rpm if is_dynamic else 0.5
        wheel_data.spin_direction = spin_direction if is_dynamic else True # Forward spin direction

        publisher_left.publish(wheel_data)
        publisher_right.publish(wheel_data)

        rate.sleep()

# callback function to handle data read from cmd_vel topic
def vel_cb(vel: Twist):
    global rpm, spin_direction
    # TODD: check the math is correct or at least logical. This is still fake data
    angular_velocity = math.sqrt(vel.linear.x^2 * vel.linear.y^2) / FAKE_WHEEL_RADUIS_IN_METERS # in radians per sec
    rpm = angular_velocity * 60 / (2 * math.pi)
    spin_direction = True if angular_velocity >= 0 else False


if __name__ == '__main__':
    fake_wheel_encoder_publisher()