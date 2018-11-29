#! /usr/bin/env python

import rospy
import math
from visualization_msgs.msg import Marker, InteractiveMarker, InteractiveMarkerControl
from interactive_markers.interactive_marker_server import *
from geometry_msgs.msg import Vector3, Point, Quaternion
from std_msgs.msg import ColorRGBA

class PoseTesterNode(object):
    def __init__(self):
        rospy.init_node("pose_tester")
        self.server = InteractiveMarkerServer("pose_tester_server")

        self.interactive_marker = self.create_interactive_marker()
         
        self.marker = self.create_sphere_marker()

        self.control = self.create_interactive_marker_control()
        self.control.markers.append(self.marker)

        self.interactive_marker.controls.append(self.control)

        self.server.insert(self.interactive_marker, self.processFeedback)
        self.server.applyChanges()

        rospy.spin()

    def create_sphere_marker(self):
        sphere_marker = Marker()

        sphere_marker.type = Marker.SPHERE
        
        sphere_marker.pose.position = Point(0, 0, 0)
        sphere_marker.pose.orientation = Quaternion(0, 0, math.sqrt(0.5), math.sqrt(0.5)) 

        sphere_marker.scale = Vector3(0.4, 0.4, 0.4)
        sphere_marker.color = ColorRGBA(0.0, 0.5, 0.5, 1.0)

        return sphere_marker
        
    def create_interactive_marker(self):
        int_marker = InteractiveMarker()

        int_marker.header.frame_id = "map"
        int_marker.name = "pose_test_marker"
        int_marker.description = "Move around for pose information"

        int_marker.pose.position = Point(0, 0, 0)
        int_marker.pose.orientation = Quaternion(0, math.sqrt(0.5), 0, math.sqrt(0.5))

        return int_marker

    def create_interactive_marker_control(self):
        control = InteractiveMarkerControl()

        control.always_visible = True
        control.interaction_mode = InteractiveMarkerControl.MOVE_PLANE

        return control
        
    def processFeedback(self, feedback):
        self.server.applyChanges()
        print(self.interactive_marker.pose)

if __name__ == "__main__":
    pose_tester = PoseTester()
