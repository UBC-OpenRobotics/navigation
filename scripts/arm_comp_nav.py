# #!/usr/bin/env python

# import rospy
# import tf
# from tf import transformations as ts

# from geometry_msgs.msg import Transform, Vector3, Quaternion
# from geometry_msgs.msg import TransformStamped
# from std_msgs.msg import Header

# if __name__ == '__main__':
#     rospy.init_node('fixed_tf_broadcaster')
#     br = tf.TransformBroadcaster()
#     listener = tf.TransformListener()
#     rate = rospy.Rate(10.0)
#     while not rospy.is_shutdown():
        
#         br.sendTransform((3.0, 2.0, 0.0),
#                          tf.transformations.quaternion_from_euler(0, 0, 45.),
#                          rospy.Time.now(),
#                          "ob1_arm_base_link",
#                          "")


#         trans = Transform(translation=Vector3(1.0, 2.0, 0),
#                           rotation=Quaternion(*tf.transformations.quaternion_from_euler(0, 0, 0))
#                         )

#         header = Header()
#         header.stamp = rospy.Time.now()
#         header.frame_id = 'torso'   

#         trans_stamp = TransformStamped(header, 'ob1_arm_base_link', trans)
#         br.sendTransformMessage(trans_stamp)

#         try:
#             (trans, rot) = listener.lookupTransform('torso', 'ob1_arm_base_link', rospy.Time(0))

#             transform = ts.concatenate_matrices(ts.translation_matrix(trans), ts.quaternion_matrix(rot))
#             inversed_trans = ts.inverse_matrix(transform)
#         except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
#             continue

#         header.frame_id = 'ob1_arm_base_link'
#         trans = Transform(translation=Vector3(*ts.translation_from_matrix(inversed_trans)),
#                           rotation=Quaternion(*ts.quaternion_from_matrix(inversed_trans))
#                         )

#         # trans_stamp_inv = TransformStamped(header, 'world_inv_py', trans)

#         # br.sendTransformMessage(trans_stamp_inv)

#         rate.sleep()

import rospy
import tf2_ros

import tf
import rospy
import tf2_ros
import time
import yaml
from geometry_msgs.msg import Vector3
from navigation.srv import NamedLocation, NamedLocationResponse
from navigation.srv import ArmAdjustment, ArmAdjustmentResponse
from navigation.srv import RelLocation, RelLocationResponse

import argparse

def adjust_base_for_arm(data):
    ########################### List all tf Nodes for debugging purposes
    if False:
        tf_buffer = tf2_ros.Buffer()
        tf_listener = tf2_ros.TransformListener(tf_buffer)
        time.sleep(5.0)

        frames_dict = yaml.safe_load(tf_buffer.all_frames_as_yaml())
        frames_list = list(frames_dict.keys())
        print(frames_list)
    ###########################
    tf_buffer = tf2_ros.Buffer()
    listener = tf.TransformListener()

    # Do not remove this sleep step!!! You need to give time for the tf transforms to register before
    # Other wise exceptions will be thrown for no frame found 
    time.sleep(1.0)

    print(f"available strings: {', '.join(listener.getFrameStrings())}")

    for i in range(10):
        err = None
        if rospy.is_shutdown(): raise rospy.ROSInterruptException()
        try:
            (trans,rot) = listener.lookupTransform(base_frame_str, arm_frame_str, rospy.Time(0))
            print(f"recieved transform: trans: {trans}, rot: {rot}")
            break
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            err = e
    else:
        raise err
    x, y, z = trans
    x, y, z = x + data.vec.x, y + data.vec.y, z + data.vec.z
    # print(x, y, z)
    vec = Vector3(x, y, z)
    rel_loc_srv(vec, 0, 0, 0)
    return ArmAdjustmentResponse(True)
        

if __name__ == '__main__':

    parser = argparse.ArgumentParser()
    parser.add_argument('--base_frame', '-b', default='base_link')
    parser.add_argument('--arm_frame', '-a', default='arm1_base_link')
    args = parser.parse_args()

    # rospy.wait_for_service('relative')
    rel_loc_srv = rospy.ServiceProxy('nav_rel_locs_server', RelLocation)

    base_frame_str = args.base_frame
    arm_frame_str = args.arm_frame

    rospy.init_node('ipython')
    s = rospy.Service('arm_adjustment', ArmAdjustment, adjust_base_for_arm)
    rospy.spin()