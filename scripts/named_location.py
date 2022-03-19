#!/usr/bin/env python3

from navigation.srv import NamedLocation, NamedLocationResponse
import rospy
from pathlib import Path
import pickle
from geometry_msgs.msg import PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib

class NamedLocationsHandler:
    def __init__(self) -> None:
        self.locations = {}
        self.locs_file = Path(rospy.get_param('named_locations_file', 'locations.pickle'))
        self.move_base_client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        self.move_base_client.wait_for_server()            
        self.load_locations_file()

    def load_locations_file(self):
        self.locs_file.touch()
        with open(self.locs_file, 'rb') as f:
            try:
                self.locations = pickle.load(f)
                print(self.locations)
            except EOFError:
                pass

    def save_locations_file(self):
        with open(self.locs_file, 'wb') as f:
            pickle.dump(self.locations, f)
            print(self.locations)

    def handle_request(self, req):
        try:
            if (req.action == "Goto"):
                goal = MoveBaseGoal()
                goal.target_pose.header.frame_id = "map"
                goal.target_pose.header.stamp = rospy.Time.now()
                # Example Move Base Action
                goal.target_pose.pose = self.locations[req.name]
                self.move_base_client.send_goal(goal)
                wait = self.move_base_client.wait_for_result()
                if not wait:
                    rospy.signal_shutdown("Move_base server unavailable!")
                else:
                # Result of executing the action
                    rospy.loginfo("Move Base Goal Executed" if self.move_base_client.get_result() else "Failed")
            elif(req.action == "Name"):
                pose_msg = rospy.wait_for_message('/amcl_pose', PoseWithCovarianceStamped)
                self.locations[req.name]=pose_msg.pose.pose
                print(f"saved location {req.name} as {pose_msg.pose.pose}")
                self.save_locations_file()
            return NamedLocationResponse(True)
        except:
            return NamedLocationResponse(False)

def add_two_ints_server():
    rospy.init_node('nav_named_locs_server')
    nl = NamedLocationsHandler()
    s = rospy.Service('/navigate_named_location', NamedLocation, nl.handle_request)
    print("Ready process locations")
    rospy.spin()

if __name__ == "__main__":
    add_two_ints_server()
