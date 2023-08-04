#!/usr/bin/env python3

import rospy 
import math
from geometry_msgs.msg import Twist
from navigation.msg import wheel_encoder_data

fake_wheel_raduis_in_meters = 0.2
fake_wheel_distance_in_meters = 0.05

rpm_left = 0
rpm_right = 0
spin_direction_left = True
spin_direction_right = True

def fake_wheel_encoder_publisher():
    global fake_wheel_raduis_in_meters, fake_wheel_distance_in_meters
    rospy.init_node("fake_wheel_encoder_publisher")
    publisher_right = rospy.Publisher("fake_wheel_encoder_publisher/right", wheel_encoder_data, queue_size=5) # change queue_size if needed
    publisher_left = rospy.Publisher("fake_wheel_encoder_publisher/left", wheel_encoder_data, queue_size=5)
    rate = rospy.Rate(1) 
    is_dynamic = rospy.get_param('dynamic', False)
    fake_wheel_raduis_in_meters = rospy.get_param('wheel_raduis', False)
    fake_wheel_distance_in_meters = rospy.get_param('wheel_distance', False)

    if(is_dynamic):
        rospy.Subscriber("cmd_vel", Twist, vel_cb)

    while not rospy.is_shutdown():
        wheel_data_right = wheel_encoder_data()
        wheel_data_right.rpm = rpm_right if is_dynamic else 0.5
        wheel_data_right.spin_direction = spin_direction_right if is_dynamic else True # Forward spin direction

        wheel_data_left = wheel_encoder_data()
        wheel_data_left.rpm = rpm_left if is_dynamic else 0.5
        wheel_data_left.spin_direction = spin_direction_left if is_dynamic else True # Forward spin direction

        publisher_left.publish(wheel_data_left)
        publisher_right.publish(wheel_data_right)

        rate.sleep()

# callback function to handle data read from cmd_vel topic
def vel_cb(vel: Twist):
    global rpm_right, rpm_left, spin_direction_right, spin_direction_left

    angular_velocity_right = (vel.linear.x + vel.angular.z * fake_wheel_distance_in_meters / 2) / fake_wheel_raduis_in_meters # in radians per sec
    rpm_right = angular_velocity_right * 60 / (2 * math.pi)
    spin_direction_right = True if angular_velocity_right >= 0 else False

    angular_velocity_left = (vel.linear.x - vel.angular.z * fake_wheel_distance_in_meters / 2) / fake_wheel_raduis_in_meters
    rpm_left = angular_velocity_left * 60 / (2 * math.pi)
    spin_direction_left = True if angular_velocity_left >= 0 else False

if __name__ == '__main__':
    fake_wheel_encoder_publisher()