#!/usr/bin/env python

import roslib, rospy, json, argparse, random
import sys
import copy
import datetime
import time
import math
import message_conversion
import object_estimation
import os
import pickle

from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray
from threading import Timer
from mongodb_store.message_store import MessageStoreProxy
from semantic_mapping.msg import SOMObservation, SOMObject
from semantic_mapping.srv import *
from std_msgs.msg import String
from visualisation import *
from object_estimation import make_observation
from ontology import Ontology
from queries import query


# Soma2 Data Manager For storing and deleting data
class SOMDataManager():
    def __init__(self, ontology_name, rois_name):

        self._object_store = MessageStoreProxy(database="som_objects", collection="objects")
        self._observation_store = MessageStoreProxy(database="som_observations", collection="observations")

        inss = rospy.Service('som/observe', SOMObserve, self.handle_observe_request)
        dels = rospy.Service('som/delete', SOMDelete, self.handle_delete_request)
        upts = rospy.Service('som/query', SOMQuery, self.handle_query_request)
        qrys = rospy.Service('som/lookup', SOMLookup, self.handle_lookup_request)

        dirname = os.path.dirname(__file__)
        fpath = os.path.join(dirname, '../config/' + rois_name)
        self._rois = pickle.load(open(fpath,"rb"))
        self._ontology = Ontology(ontology_name)
        draw_rois(self._rois)

        rospy.spin()

    # Handles the soma2 objects to be inserted
    def handle_observe_request(self,req):
        obs = req.observation
        res, id = make_observation(obs, self._object_store, self._observation_store)
        return SOMObserveResponse(res, id)

    # Handles the delete request of soma2 objs
    def handle_delete_request(self,req):
        oid = req.obj_id
        res = self._object_store.query(SOMObject._type,message_query={"obj_id": oid})

        for o,om in res:
            try:
                self._object_store.delete(str(om['obj_id']))
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
    myargv = rospy.myargv(argv=sys.argv)
    if  len(myargv) < 3:
        print('usage: som_manager.py <ontology.owl> <rois.pkl>')
    else:
        SOMDataManager(myargv[1], myargv[2])
