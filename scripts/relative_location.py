#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Point

from navigation.srv import RelativeLocation, RelativeLocationResponse
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

import rospy
import tf2_ros

import tf
import rospy
import tf2_ros
import tf2_geometry_msgs
import time
import yaml


def handle_request(data):
    try:
        transform = tf_buffer.lookup_transform('odom', 'base_link', rospy.Time(0)) # rospy.Time(0) gets latest transform
        pose_stamped = PoseStamped()
        pose_stamped.pose.position.x = data.x
        pose_stamped.pose.position.y = data.y
        pose_transformed = tf2_geometry_msgs.do_transform_pose(pose_stamped, transform)
        pub.publish(pose_transformed)

        return RelativeLocationResponse(True)
    except:
        return RelativeLocationResponse(False)

if __name__ == "__main__":
    rospy.init_node('nav_rel_locs_server')

    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    time.sleep(1.0) # wait for transform buffer to at least hear one message
    frames_dict = yaml.safe_load(tf_buffer.all_frames_as_yaml())
    frames_list = list(frames_dict.keys())
    if 'odom' not in frames_list: raise rospy.ServiceException('No odom frame')
    if 'base_link' not in frames_list: raise rospy.ServiceException('No base_link frame')
    pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)


    s = rospy.Service('/navigate_relative_location', RelativeLocation, handle_request)
    print("Ready to process relative locations")
    rospy.spin()

    
    