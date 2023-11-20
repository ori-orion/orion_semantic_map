"""
Adapted from Marc Rigter's system.

Author: Marc Rigter, Matthew Munks
Owner: Matthew Munks
"""

import math
from typing import Callable, Optional

from bson import ObjectId
import rospy
from new_version.DatabaseTypes import (
    DbObject,
    DbPose,
    convertDictToPoint,
    convertDictToPose,
)
from new_version.constants import MAP_FRAME, PYMONGO_ID_KEY
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Pose
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl
from orion_actions.msg import SOMObservation, SOMObject

# import CollectionManager;
import pymongo.collection
import utils


class RvizVisualisationManager:
    def __init__(
        self,
        im_server: InteractiveMarkerServer,
        colour_a: float,
        colour_r: float,
        colour_g: float,
        colour_b: float,
        class_attr: str,
        size_attr: str,
        position_attr: str,
    ):
        # Interactive Marker server.
        self.im_server: InteractiveMarkerServer = im_server
        # self.coll_manager:CollectionManager.CollectionManager = coll_manager;

        # In the range [0,1]
        self.colour_a = colour_a
        self.colour_r = colour_r
        self.colour_g = colour_g
        self.colour_b = colour_b

        self.class_attr = class_attr
        self.size_attr = size_attr
        self.position_attr = position_attr

        self.query_callback: Optional[Callable[[dict], list]] = None

    def delete_object(self, name: str):
        """
        Deletes the rviz object with id.
        """
        self.im_server.erase(name)
        self.im_server.applyChanges()

    def handle_viz_input(self, input):
        """
        Handles the click callback for when the user clicks on one of the rviz boxes.
        """
        # TODO: Fix this callbacks to somethin better
        if self.query_callback != None:
            obj: list = self.query_callback(
                {PYMONGO_ID_KEY: ObjectId(input.marker_name)}
            )

            if len(obj) > 0:
                rospy.loginfo(obj)
                rospy.loginfo("\n\n\n")

    def add_obj_dict(
        self, obj_to_add: DbObject, obj_id: ObjectId, num_observations: int = 10_000_000
    ):
        """
        This acts both to add and to update a given entry.
        """
        obj_pose = Pose()

        if "position" in obj_to_add[self.size_attr]:
            carry: Pose = convertDictToPose(obj_to_add[self.size_attr])
            obj_size = carry.position
            obj_pose.orientation = carry.orientation
        else:
            # We assume that the size_attr will be a dict of type DbVec3D
            obj_size = convertDictToPoint(obj_to_add[self.size_attr])

        if "position" in  obj_to_add[self.position_attr]:
            obj_pose = convertDictToPose(obj_to_add[self.position_attr])
        else:
            obj_pose.orientation.w = 1
            obj_pose.position = convertDictToPoint(obj_to_add[self.position_attr])

        obj_class = obj_to_add[self.class_attr]

        self.add_object(str(obj_id), obj_pose, obj_size, obj_class, num_observations)

    def add_object(
        self,
        name: str,
        pose: Pose,
        size: Point,
        obj_class: str,
        num_observations: int,
    ):
        """
        Deals with the adding of an object to the visualisation server.
        name                  - The id of the object (and the name that rviz will use).
        pose                - The pose of the object
        size                - The size of the object
        obj_class           - The label the object will be given in the visualisation.
        num_observations    - The number of observations (will be used in a function for the alpha value of the object.)
        """

        self.im_server.erase(name)

        box_marker = Marker()
        box_marker.type = Marker.CUBE
        box_marker.pose.orientation.w = 1

        min_box_size = 0.05
        box_marker.scale.x = max(min_box_size, size.x)
        box_marker.scale.y = max(min_box_size, size.y)
        box_marker.scale.z = max(min_box_size, size.z)

        box_marker.color.r = self.colour_r
        box_marker.color.g = self.colour_g
        box_marker.color.b = self.colour_b
        box_marker.color.a = self.colour_a * num_observations / 10
        if box_marker.color.a > self.colour_a:
            box_marker.color.a = self.colour_a

        button_control = InteractiveMarkerControl(
            interaction_mode=InteractiveMarkerControl.BUTTON,
            always_visible=True,
            markers=[box_marker],
        )

        int_marker = InteractiveMarker(
            name=name, description=obj_class, pose=pose, controls=[button_control]
        )
        int_marker.header.frame_id = MAP_FRAME

        self.im_server.insert(int_marker, self.handle_viz_input)  # type: ignore
        self.im_server.applyChanges()
