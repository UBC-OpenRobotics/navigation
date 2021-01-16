#!/usr/bin/env python  
import rospy

# Because of transformations
import tf2_ros



def broadcastTransform(turtlename, frame_id, arr):
    br = tf2_ros.TransformBroadcaster()

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = frame_id
    t.child_frame_id = turtlename
    t.transform.translation.x = arr[0]
    t.transform.translation.y = arr[1]
    t.transform.translation.z = arr[2]
    t.transform.rotation.x = arr[3]
    t.transform.rotation.y = arr[4]
    t.transform.rotation.z = arr[5]
    t.transform.rotation.w = arr[6]

    br.sendTransform(t)

if __name__ == '__main__':
    rospy.init_node('tf2_turtle_broadcaster')
    turtlename = rospy.get_param('~turtle')
	frame_id = rospy.get_param('~frame')
	transform = rospy.get_param('~transform')
    broadcastTransform(turtlename, frame_id, transform)
    rospy.spin()
