#!/usr/bin/env python

import roslib, rospy, json, argparse, random
import sys
import copy
import datetime
import time
import math
import message_conversion
import os
import pickle
import visualisation

from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray
from threading import Timer
from mongodb_store.message_store import MessageStoreProxy
from std_msgs.msg import String
from observation import make_observation
from ontology import Ontology
from queries import query
from queries import read_prior_csv
from queries import get_prior_probs
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback, Marker, MarkerArray
from orion_actions.msg import *
from orion_actions.srv import *
from interactive_markers.interactive_marker_server import *

# this line is needed to make sure we can load the pickle files
dirname = os.path.dirname(__file__)
fpath = os.path.join(dirname, '../../../../orion_actions/orion_actions/msg')
sys.path.append('fpath')
prior_path = os.path.join(dirname, "../..", "config", "priors.csv")

# Soma2 Data Manager For storing and deleting data
class SOMDataManager():
    def __init__(self, ontology_name, rois_name):
        rospy.init_node("soma_data_manager")
        self._ontology = Ontology(ontology_name)
        self._object_store = MessageStoreProxy(database="som_objects", collection="objects")
        self._observation_store = MessageStoreProxy(database="som_observations", collection="observations")
        self._prior_knowledge_df = read_prior_csv(prior_path)

        roi_vis_pub = rospy.Publisher('som/roi_vis', MarkerArray, queue_size=1, latch=True)
        self.server = InteractiveMarkerServer("som/obj_vis")

        dirname = os.path.dirname(__file__)
        fpath = os.path.join(dirname, '../../config/' + rois_name)
        roi_load = pickle.load(open(fpath,"rb"))
        self._rois = [i[0] for i in roi_load]
        roi_markers = visualisation.rois_to_marker_array(self._rois)
        roi_vis_pub.publish(roi_markers)

        obss = rospy.Service('som/observe', SOMObserve, self.handle_observe_request)
        dels = rospy.Service('som/delete', SOMDelete, self.handle_delete_request)
        qrys = rospy.Service('som/query', SOMQuery, self.handle_query_request)
        lkps = rospy.Service('som/lookup', SOMLookup, self.handle_lookup_request)
        clr = rospy.Service('som/clear_database', SOMClearDatabase, self.clear_databases)
        getall = rospy.Service('som/get_all_objects', SOMGetAllObjects, self.get_all_objects)
        rospy.spin()

    # Handles the soma2 objects to be inserted
    def handle_observe_request(self,req):
        obs = req.observation
        #if (not self._ontology.check_class_exists(obs.type)) and (obs.type != ''):
        #    raise Exception('Type specified in observation is not valid ontology class')
        #    return SOMObserveResponse(False, '')
        res, id, obj = make_observation(obs, self._rois, self._object_store, self._observation_store)

        if res:
            visualisation.update_objects(obj, id, self.server)
        return SOMObserveResponse(res, id)

    def get_all_objects(self, req):
        resp = self._object_store.query(SOMObject._type)
        objects = [i[0] for i in resp]
        return SOMGetAllObjectsResponse(objects)

    def clear_databases(self, req):
        objs = self._object_store.query(SOMObject._type)
        obss = self._observation_store.query(SOMObservation._type)
        for object,meta in objs:
            self._object_store.delete(str(meta['_id']))
            visualisation.delete_object(str(meta['_id']), self.server)
        for obs,meta in obss:
            self._observation_store.delete(str(meta['_id']))
        return SOMClearDatabaseResponse()

    # Handles the delete request of soma2 objs
    def handle_delete_request(self,req):
        obj_id = req.obj_id
        response = self._observation_store.query(SOMObject._type,message_query={"obj_id": obj_id})

        try:
            self._object_store.delete(obj_id)
            visualisation.delete_object(obj_id, self.server)
        except:
            return SOMDeleteResponse(False)

        for obj, meta in response:
            try:
                self._observation_store.delete(meta['_id'])
            except:
                return SOMDeleteResponse(False)
        return SOMDeleteResponse(True)

    # Handles the soma2 objects to be inserted
    def handle_query_request(self,req):
        som_template_one = req.obj1
        relation = req.relation
        som_template_two = req.obj2
        cur_robot_pose = req.current_robot_pose
        matches = query(som_template_one, relation, som_template_two, cur_robot_pose, self._object_store, self._prior_knowledge_df, self._rois,  self._ontology)
        return SOMQueryResponse(matches)

    def handle_lookup_request(self, req):
        try:
            obj, meta = self._object_store.query_id(req.obj_id, SOMObject._type)
        except rospy.ServiceException, e:
            obj = None
            print("Service call failed: %s" % (e))
        return obj

if __name__=="__main__":
    myargv = rospy.myargv(argv=sys.argv)
    if  len(myargv) < 3:
        print('usage: som_manager.py <ontology.owl> <rois.pkl>')
    else:
        SOMDataManager(myargv[1], myargv[2])
