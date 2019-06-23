#!/usr/bin/env python

import roslib, rospy, json, argparse, random, os
import copy, sys, datetime, time, math

from geometry_msgs.msg import Pose, Point
from geometry_msgs.msg import PoseArray
from threading import Timer
from mongodb_store.message_store import MessageStoreProxy
from orion_actions.msg import SOMObservation, SOMObject, Relation
from orion_actions.srv import *
from std_msgs.msg import String

def test_database():
    rospy.wait_for_service('som/observe')
    rospy.wait_for_service('som/lookup')
    rospy.wait_for_service('som/delete')
    rospy.wait_for_service('som/query')
    rospy.wait_for_service('som/clear_database')
    rospy.wait_for_service('som/check_similarity')
    rospy.wait_for_service('som/get_all_objects')

    observe_objs_srv = rospy.ServiceProxy('som/observe', SOMObserve)
    lookup_object_srv = rospy.ServiceProxy('som/lookup', SOMLookup)
    delete_object_srv = rospy.ServiceProxy('som/delete', SOMDelete)
    query_object_srv = rospy.ServiceProxy('som/query', SOMQuery)
    clear_database_srv = rospy.ServiceProxy('som/clear_database', SOMClearDatabase)
    check_similarity_srv = rospy.ServiceProxy('som/clear_database', SOMClearDatabase)
    get_all_objects_srv = rospy.ServiceProxy('som/get_all_objects', SOMGetAllObjects)

    # clear out the database
    clear_database_srv()

    # observe some bacon
    similarity = check_similarity_srv(SOMObservation(type="pizza"), SOMObservation(type="milk"))

if __name__=="__main__":
    test_database()
