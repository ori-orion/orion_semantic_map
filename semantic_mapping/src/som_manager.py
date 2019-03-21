#!/usr/bin/env python

import roslib, rospy, json, argparse, random
import copy, sys, datetime, time, math

from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray
from threading import Timer
from mongodb_store.message_store import MessageStoreProxy
from semantic_mapping.msg import SOMROIObject, SOMObject
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

        inss = rospy.Service('som/insert_objects', SOMInsertObjs, self.handle_insert_request)
        dels = rospy.Service('som/delete_objects', SOMDeleteObjs, self.handle_delete_request)
        upts = rospy.Service('som/update_object', SOMUpdateObject, self.handle_update_request)
        qrys = rospy.Service('som/query_object', SOMQueryObject, self.handle_query_request)

        rospy.spin()

    # Handles the soma2 objects to be inserted
    def handle_insert_request(self,req):
        _ids = list()
        for obj in req.objects:

          if(obj.logtimestamp == 0):
            obj.logtimestamp = rospy.Time.now().secs

          d = datetime.datetime.utcfromtimestamp(obj.logtimestamp)
          obj.loghour = d.hour
          obj.logminute = d.minute
          obj.logday = d.isoweekday()
          obj.logtimeminutes = obj.loghour*60 + obj.logminute

          if (obj.header.frame_id == ""):
              obj.header.frame_id = "/map"

          if(obj.cloud.header.frame_id == ""):
              obj.cloud.header.frame_id = "/map"

          try:
                _id = self._message_store.insert(obj)
                _ids.append(_id)

          except:
                return SOMInsertObjsResponse(False,_ids)

        return SOMInsertObjsResponse(True,_ids)


    # Handles the delete request of soma2 objs
    def handle_delete_request(self,req):

        for oid in req.ids:
            res = self._message_store.query(SOMObject._type,message_query={"id": oid})
            #print len(res)
            for o,om in res:
                try:
                    self._message_store.delete(str(om['_id']))
                except:
                      return SOMDeleteObjsResponse(False)

        return SOMDeleteObjsResponse(True)

    # Handles the soma2 objects to be inserted
    def handle_update_request(self,req):

        obj = req.object


        if(obj.logtimestamp == 0):
            obj.logtimestamp = rospy.Time.now().secs

        d = datetime.datetime.utcfromtimestamp(obj.logtimestamp)
        obj.loghour = d.hour
        obj.logminute = d.minute
        obj.logday = d.isoweekday()
        obj.logtimeminutes = obj.loghour*60 + obj.logminute

        if (obj.header.frame_id == ""):
            obj.header.frame_id = "/map"

        if(obj.cloud.header.frame_id == ""):
            obj.cloud.header.frame_id = "/map"

        try:
            self._message_store.update_id(req.db_id, obj)
        except:
            return SOMUpdateObjectResponse(False)

        return SOMUpdateObjectResponse(True)


    def handle_query_request(self, req):
        try:
            stored_object, meta = self._message_store.query_id(req.id, SOMObject._type)

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
