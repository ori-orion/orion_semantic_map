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
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback
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
        self.im_server.erase(id);
        self.im_server.applyChanges();

    def handle_viz_input(self, input):
        if (self.query_callback != None):
            obj:list = self.query_callback(
                {utils.PYMONGO_ID_SPECIFIER:pymongo.collection.ObjectId(input.marker_name)});

            if len(obj) > 0:
                rospy.loginfo(obj);
                rospy.loginfo("\n\n\n");

    # NOTE: Assumption: the object position is a pose.
    # This acts both to add and to update a given entry.
    def add_obj_dict(self, adding_dict:dict, obj_id:str, num_observations=math.inf):
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
        if obj_class != "person":
            return;

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


# class Visualisation(object):
#     def __init__(self, coll_manager:CollectionManager.CollectionManager, server:InteractiveMarkerServer):
#         self.coll_manager:CollectionManager.CollectionManager = coll_manager
#         self.server:InteractiveMarkerServer = server

#     def handle_viz_input(self, input):
#         if (input.event_type == InteractiveMarkerFeedback.BUTTON_CLICK):
#             obj = self.coll_manager.queryIntoCollection(
#                 {utils.PYMONGO_ID_SPECIFIER:pymongo.collection.ObjectId(input.marker_name)});
#             rospy.loginfo(obj);
#             rospy.loginfo("\n\n\n");

#     def delete_object(self, id):
#         self.server.erase(id);
#         self.server.applyChanges();

#     def update_objects(self, object:SOMObject, id):

#         # delete marker from object if its already there
#         self.server.erase(id)

#         int_marker = InteractiveMarker()
#         int_marker.header.frame_id = "map"
#         int_marker.name = id
#         int_marker.description = object.class_
#         int_marker.pose.position.x = object.obj_position.position.x;
#         int_marker.pose.position.y = object.obj_position.position.y;
#         int_marker.pose.position.z = object.obj_position.position.z;
#         int_marker.pose.orientation.w = 1

#         box_marker = Marker()
#         box_marker.type = Marker.CUBE
#         box_marker.pose.orientation.w = 1
#         if object.size.x > 0.05:
#             box_marker.scale.x = object.size.x
#         else:
#             box_marker.scale.x = 0.05
#         if object.size.y > 0.05:
#             box_marker.scale.y = object.size.y
#         else:
#             box_marker.scale.y = 0.05
#         if object.size.z > 0.05:
#             box_marker.scale.z = object.size.z
#         else:
#             box_marker.scale.z = 0.05
#         box_marker.color.r = 0.0
#         box_marker.color.g = 0.2
#         box_marker.color.b = 1.0
#         box_marker.color.a = 0.7

#         button_control = InteractiveMarkerControl()
#         button_control.interaction_mode = InteractiveMarkerControl.BUTTON
#         button_control.always_visible = True
#         button_control.markers.append(box_marker);
#         int_marker.controls.append(button_control);

#         self.server.insert(int_marker, self.handle_viz_input)
#         self.server.applyChanges();
#         return

    # def rois_to_marker_array(self, rois):
    #     markers = []
    #     for roi in rois:
    #         points = []
    #         poses = roi.posearray.poses
    #         total_x = 0.0
    #         total_y = 0.0
    #         total_z = 0.0
    #         for pose in poses:
    #             total_x = total_x + pose.position.x
    #             total_y = total_y + pose.position.y
    #             total_z = total_z + pose.position.z
    #         avg_x = total_x/len(poses)
    #         avg_y = total_y/len(poses)
    #         avg_z = total_z/len(poses)
    #         poses.append(poses[0])
    #         for pose in poses:
    #             points.append(Point(pose.position.x, pose.position.y, pose.position.z))
    #         marker = Marker()
    #         marker.header.frame_id = "map"
    #         marker.header.stamp = rospy.Time.now()
    #         marker.ns = "region"
    #         marker.id = len(markers)
    #         marker.type = 4
    #         marker.scale.x = 0.07
    #         marker.color.a = 1.0
    #         marker.color.r = 0.6
    #         marker.points = points
    #         marker.text = roi.name

    #         text_marker = Marker()
    #         text_marker.header.frame_id = "map"
    #         text_marker.header.stamp = rospy.Time.now()
    #         text_marker.ns = "text"
    #         text_marker.id = len(markers)
    #         text_marker.type = 9
    #         text_marker.scale.z = 0.15
    #         text_marker.color.a = 1.0
    #         text_marker.color.r = 1.0
    #         text_marker.color.g = 1.0
    #         text_marker.color.b = 0.8
    #         pose = Pose()
    #         pose.position.x = avg_x
    #         pose.position.y = avg_y
    #         pose.position.z = avg_z + 0.1
    #         text_marker.pose = pose

    #         text_marker.text = roi.name
    #         markers.append(text_marker)
    #         markers.append(marker)
    #     return markers
