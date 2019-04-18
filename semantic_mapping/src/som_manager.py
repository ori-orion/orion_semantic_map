#!/usr/bin/env python

import roslib, rospy, json, argparse, random
import copy, sys, datetime, time, math

from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray
from threading import Timer
from mongodb_store.message_store import MessageStoreProxy
from semantic_mapping.msg import SOMObservation, SOMObject
from semantic_mapping.srv import *
from std_msgs.msg import String


# Soma2 Data Manager For storing and deleting data
class SOMDataManager():

    def __init__(self):


        # Initialize the mongodb proxy
        self._object_store = MessageStoreProxy(database="som_objects", collection="objects")
        self._observation_store = MessageStoreProxy(database="som_observations", collection="observations")

        inss = rospy.Service('som/observe', SOMObserve, self.handle_observe_request)
        dels = rospy.Service('som/delete', SOMDelete, self.handle_delete_request)
        upts = rospy.Service('som/query', SOMQuery, self.handle_query_request)
        qrys = rospy.Service('som/lookup', SOMLookup, self.handle_lookup_request)

        rospy.spin()

    # Handles the soma2 objects to be inserted
    def handle_observe_request(self,req):
        obj_ids = list()

        for obs in req.observations:

            # create a SOM object from the SOM observation
            time_now = rospy.Time.now().secs
            #obs.timestamp = time_now
            obj = SOMObject()

            if (obj.header.frame_id == ""):
                obj.header.frame_id = "/map"

            if(obj.cloud.header.frame_id == ""):
                obj.cloud.header.frame_id = "/map"

            obj.map_name = obs.map_name
            obj.meta_properties = obs.meta_properties
            obj.type = obs.type
            obj.size = obs.size
            obj.weight = obs.weight
            obj.task_role = obs.task_role
            obj.robot_pose = obs.robot_pose
            obj.cloud = obs.cloud
            obj.colour = obs.colour
            obj.room_name = obs.room_name
            obj.waypoint = obs.waypoint
            obj.room_geometry = obs.room_geometry
            obj.name = obs.name
            obj.age = obs.age
            obj.posture = obs.posture
            obj.gender = obs.gender
            obj.shirt_colour = obs.shirt_colour

            ## if no object id is supplied insert new object
            if obs.obj_id == "":
                try:
                    obj_id = self._object_store.insert(obj)
                    obj_ids.append(obj_id)

                    # add new observation to observation store
                    obs.obj_id = obj_id
                    obs_id = self._observation_store.insert(obs)

                except rospy.ServiceException, e:
                    print("Service call failed: %s"%(e))
                    return SOMObserveResponse(False, obj_ids)

            ## if the object id is supplied then update existing
            else:
                try:

                    self._object_store.update_id(obs.obj_id, obj)
                    obj_ids.append(obs.obj_id)

                    obs.obj_id = obs.obj_id
                    obs_id = self._observation_store.insert(obs)

                except:
                    return SOMObserveResponse(False, obj_ids)

        return SOMObserveResponse(True, obj_ids)


    # Handles the delete request of soma2 objs
    def handle_delete_request(self,req):

        for oid in req.obj_ids:
            res = self._object_store.query(SOMObject._type,message_query={"obj_id": oid})

            for o,om in res:
                try:
                    self._object_store.delete(str(om['_id']))
                except:
                      return SOMDeleteResponse(False)

        return SOMDeleteResponse(True)

    # Handles the soma2 objects to be inserted
    def handle_query_request(self,req):

        my_query = {"type": "shirt", "colour":"red"}
        print self._object_store.query(SOMObject._type, my_query)

        return SOMQueryResponse([])


    def handle_lookup_request(self, req):
        try:
            obj, meta = self._object_store.query_id(req.obj_id, SOMObject._type)

        except rospy.ServiceException, e:
            obj = None
            print("Service call failed: %s" % (e))

        return obj

if __name__=="__main__":
    rospy.init_node("soma_data_manager")
    SOMDataManager()
