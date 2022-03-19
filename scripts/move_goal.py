#!/usr/bin/env python3

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

def movebase_client():

   # create MoveBaseGoal and send to move_base server node 
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    # Example Move Base Action
    goal.target_pose.pose.position.x = 0.6094982059607833
    goal.target_pose.pose.position.y = -0.39999928174738597
    goal.target_pose.pose.position.z = 0.0 
    goal.target_pose.pose.orientation.w = 0.9063164541178897
    goal.target_pose.pose.orientation.z = 0.4225996746273889
    client.send_goal(goal)
    wait = client.wait_for_result()
   # If the result doesn't arrive, assume the Server is not available
    if not wait:
        rospy.signal_shutdown("Move_base server unavailable!")
    else:
    # Result of executing the action
        return client.get_result()   

# If the python node is executed as main process (sourced directly)
if __name__ == '__main__':
    try:
        rospy.init_node('movebase_client_py')

        # Create an action client for move_base server
        client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        # Waits until the action server has started up
        client.wait_for_server()

        result = movebase_client()
        if result:
            rospy.loginfo("Move Base Goal Executed")
    except rospy.ROSInterruptException:
        pass