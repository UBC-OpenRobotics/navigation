#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

from tf import transformations
import tf
import numpy as np
from geometry_msgs.msg import PoseStamped, Pose, Point, PointStamped, Quaternion, Twist, TransformStamped
from nav_msgs.msg import Odometry

from navigation.msg import wheel_encoder_data
from fake_wheel_encoder_publisher import fake_wheel_raduis_in_meters, fake_wheel_distance_in_meters

v_lin = 0
v_th = 0

left_rpm = 0
right_rpm = 0

def vel_cb(msg:Twist):
    global v_lin, v_th
    v_lin = msg.linear.x
    v_th = msg.angular.z

def process_encoder(data: wheel_encoder_data, is_right_wheel: bool):
    # https://answers.ros.org/question/306140/actual-wheels-rpm-velocities-from-ticks-target-velocity-and-robot-velocity/
    global left_rpm, right_rpm, v_lin, v_th
    rpm = data.rpm if data.spin_direction else -data.rpm
    if is_right_wheel:
        right_rpm = rpm
    else:
        left_rpm = rpm
    
    v_lin = (left_rpm + right_rpm) / 120 * (fake_wheel_raduis_in_meters * 2) * np.pi
    v_th = ((right_rpm - left_rpm) / 120 * (fake_wheel_raduis_in_meters * 2) * np.pi) / fake_wheel_distance_in_meters

def odom_publisher():
    rospy.init_node("ob1_odometry_publisher")

    odom_publisher = rospy.Publisher("odom",Odometry, queue_size=5) # arbitrary queue_value
    odom_tf_broadcaster = tf.TransformBroadcaster()

    rospy.Subscriber("fake_wheel_encoder_publisher/left", wheel_encoder_data, process_encoder, False)
    rospy.Subscriber("fake_wheel_encoder_publisher/right", wheel_encoder_data, process_encoder, True)

    t_current = rospy.Time.now()
    t_last = rospy.Time.now()

    r = rospy.Rate(1)

    x = 0
    y = 0
    th = 0

    while not rospy.is_shutdown():
        t_current = rospy.Time.now()
        dt = t_current.to_sec() - t_last.to_sec()

        dx = v_lin * np.cos(th) * dt
        dy = v_lin * np.sin(th) * dt
        dth = v_th * dt

        x = x + dx
        y = y + dy
        th = th + dth

        q_odom = transformations.quaternion_from_euler(0,0,th)
        q_odom = Quaternion(q_odom[1],q_odom[2],q_odom[3],q_odom[0])

        #####Transfrom message#####

        tfs_odom = TransformStamped()

        #stamp
        tfs_odom.header.stamp = t_current
        tfs_odom.header.frame_id = "odom" 
        tfs_odom.child_frame_id = "base_footprint"

        #position and orientation
        tfs_odom.transform.translation.x = x
        tfs_odom.transform.translation.y = y
        tfs_odom.transform.translation.z = 0
        tfs_odom.transform.rotation = q_odom

        #send tf
        odom_tf_broadcaster.sendTransformMessage(tfs_odom)

        ####Odom message####
        odom = Odometry()
        odom.header.stamp = t_current
        odom.header.frame_id = "odom"

        
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = 0
        odom.pose.pose.orientation = q_odom

        odom.child_frame_id = "base_footprint"
        odom.twist.twist.linear.x = v_lin
        odom.twist.twist.angular.z = v_th

        odom_publisher.publish(odom)

        t_last = t_current

        r.sleep()

if __name__ == '__main__':
    print("________________Starting up Odom publisher________________")
    odom_publisher()



















