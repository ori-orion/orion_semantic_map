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

    def __init__(self, db_name="som_data", collection_name="som"):

       # self.soma_map_name = soma_map_name
        self._db_name = db_name
        self._collection_name = collection_name

        # Initialize the mongodb proxy
        self._message_store = MessageStoreProxy(database=db_name, collection=collection_name)

        inss = rospy.Service('som/observe', SOMObserve, self.handle_observe_request)
        dels = rospy.Service('som/delete', SOMDelete, self.handle_delete_request)
        upts = rospy.Service('som/query', SOMQuery, self.handle_query_request)
        qrys = rospy.Service('som/lookup', SOMLookup, self.handle_lookup_request)

        rospy.spin()

    # Handles the soma2 objects to be inserted
    def handle_observe_request(self,req):
        _ids = list()
        for obj in req.objects:

            # add timestamp and frame headers
            obj.timestamp = rospy.Time.now().secs

            if (obj.header.frame_id == ""):
                obj.header.frame_id = "/map"

            if(obj.cloud.header.frame_id == ""):
                obj.cloud.header.frame_id = "/map"

            ## if no object id is supplied insert new object
            if obj.id == "":
                try:
                    _id = self._message_store.insert(obj)
                    _ids.append(_id)

                except:
                    return SOMObserveResponse(False,_ids)

            ## if the object id is supplied then update existing
            else:
                try:
                    self._message_store.update_id(req.db_id, obj)
                    _ids.append(req.db_id)
                except:
                    return SOMObserveResponse(False,_ids)

        return SOMObserveResponse(True,_ids)


    # Handles the delete request of soma2 objs
    def handle_delete_request(self,req):

        for oid in req.ids:
            res = self._message_store.query(SOMObject._type,message_query={"id": oid})

            for o,om in res:
                try:
                    self._message_store.delete(str(om['_id']))
                except:
                      return SOMDeleteResponse(False)

        return SOMDeleteResponse(True)

    # Handles the soma2 objects to be inserted
    def handle_query_request(self,req):

        dostuff = 5

    def handle_lookup_request(self, req):
        try:
            stored_object, meta = self._message_store.query_id(req.id, SOMObservation._type)

        except rospy.ServiceException, e:
            stored_object = None
            print("Service call failed: %s" % (e))

        return stored_object

if __name__=="__main__":

    parser = argparse.ArgumentParser(prog='data_manager.py')
    parser.add_argument("db_name", nargs='?', help='Name of the database')
    parser.add_argument('collection_name', nargs='?', help='Name of the collection')

    args = parser.parse_args(rospy.myargv(argv=sys.argv)[1:])

    rospy.init_node("soma_data_manager")
    if args.db_name is not None:
    	if args.collection_name is not None:
            rospy.loginfo("Running SOM data manager (dbname: %s, collection_name: %s)", args.db_name, args.collection_name)
            SOMDataManager(args.db_name,args.collection_name)
    	else:
            rospy.loginfo("Running SOM data manager (dbname: %s, collection_name: soma)", args.db_name)
            SOMDataManager(args.db_name)
    else:
        rospy.loginfo("Running SOM data manager (dbname: somdata, collection_name: soma)")
        SOMDataManager()
