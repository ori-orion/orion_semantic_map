"""
Adapted from Marc Rigter's system.

Author: Marc Rigter, Matthew Munks
Owner: Matthew Munks
"""

import math;
import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Pose
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl
from orion_actions.msg import SOMObservation, SOMObject

# import CollectionManager;
import pymongo.collection;
import utils;


class RvizVisualisationManager:
    def __init__(self, 
        im_server:InteractiveMarkerServer, 
        colour_a, colour_r, colour_g, colour_b,
        class_attr:str, size_attr:str, position_attr:str):

        # Interactive Marker server.
        self.im_server:InteractiveMarkerServer = im_server;
        # self.coll_manager:CollectionManager.CollectionManager = coll_manager;

        # In the range [0,1]
        self.colour_a = colour_a;
        self.colour_r = colour_r;
        self.colour_g = colour_g;
        self.colour_b = colour_b;

        self.class_attr = class_attr;
        self.size_attr = size_attr;
        self.position_attr = position_attr;

        self.query_callback = None;

    def delete_object(self, id):
        """
        Deletes the rviz object with id.
        """
        self.im_server.erase(id);
        self.im_server.applyChanges();

    def handle_viz_input(self, input):
        """
        Handles the click callback for when the user clicks on one of the rviz boxes.
        """
        if (self.query_callback != None):
            obj:list = self.query_callback(
                {utils.PYMONGO_ID_SPECIFIER:pymongo.collection.ObjectId(input.marker_name)});

            if len(obj) > 0:
                rospy.loginfo(obj);
                rospy.loginfo("\n\n\n");

    def add_obj_dict(self, adding_dict:dict, obj_id:str, num_observations=math.inf):
        """
        This acts both to add and to update a given entry.
        """
        obj_pose = Pose();
        obj_pose.orientation.w=1;

        if "position" in adding_dict[self.size_attr]:
            carry:Pose = utils.dict_to_obj(adding_dict[self.size_attr], Pose());
            obj_size = carry.position;
            obj_pose.orientation = carry.orientation;
        else:
            obj_size = utils.dict_to_obj(adding_dict[self.size_attr], Point());
        
        if "position" in adding_dict[self.position_attr]:
            obj_pose = utils.dict_to_obj(adding_dict[self.position_attr], Pose());
        else:
            obj_pose.position = utils.dict_to_obj(adding_dict[self.position_attr], Point());
        
        obj_class = adding_dict[self.class_attr];

        self.add_object(obj_id, obj_pose, obj_size, obj_class, num_observations);

    def add_object(self, id:str, pose:Pose, size:Point, obj_class:str, num_observations=math.inf):
        """
        Deals with the adding of an object to the visualisation server. 
        id                  - The id of the object (and the id that rviz will use).
        pose                - The pose of the object
        size                - The size of the object
        obj_class           - The label the object will be given in the visualisation.
        num_observations    - The number of observations (will be used in a function for the alpha value of the object.)
        """
        self.im_server.erase(id);

        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "map"
        int_marker.name = id;
        int_marker.description = obj_class;
        int_marker.pose = pose;

        box_marker = Marker();
        box_marker.type = Marker.CUBE;
        box_marker.pose.orientation.w = 1;
        
        box_marker.scale.x = size.x if size.x > 0.05 else 0.05;
        box_marker.scale.y = size.y if size.y > 0.05 else 0.05;
        box_marker.scale.z = size.z if size.z > 0.05 else 0.05;
        
        box_marker.color.r = self.colour_r;
        box_marker.color.g = self.colour_g;
        box_marker.color.b = self.colour_b;
        box_marker.color.a = self.colour_a * num_observations/100;
        if box_marker.color.a > self.colour_a:
            box_marker.color.a = self.colour_a;

        button_control = InteractiveMarkerControl()
        button_control.interaction_mode = InteractiveMarkerControl.BUTTON
        button_control.always_visible = True
        button_control.markers.append(box_marker);
        int_marker.controls.append(button_control);

        self.im_server.insert(int_marker, self.handle_viz_input)
        self.im_server.applyChanges();
        return