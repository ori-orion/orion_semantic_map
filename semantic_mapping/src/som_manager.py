#!/usr/bin/env python

import roslib, rospy, json, argparse, random
import copy, sys, datetime, time, math

import message_conversion

from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray
from threading import Timer
from mongodb_store.message_store import MessageStoreProxy
from semantic_mapping.msg import SOMObservation, SOMObject
from semantic_mapping.srv import *
from std_msgs.msg import String

from queries import query


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
            time_now = rospy.Time.now().secs
            obs.timestamp = time_now
            obj = message_conversion.observation_to_object(obs, default_frame_id="/map")

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
        som_template_one = req.x
        relation = req.relation
        som_template_two = req.y
        cur_robot_pose = req.current_robot_pose
        matches = query(som_template_one, relation, som_template_two, cur_robot_pose, self._object_store)
        return SOMQueryResponse(matches)

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
