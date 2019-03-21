#!/usr/bin/env python

import roslib, rospy, json, argparse, random
import copy, sys, datetime, time, math

from geometry_msgs.msg import Pose, Point
from geometry_msgs.msg import PoseArray
from threading import Timer
from mongodb_store.message_store import MessageStoreProxy
from semantic_mapping.msg import SOMROIObject, SOMObject
from semantic_mapping.srv import *
from std_msgs.msg import String

def test_database():
    rospy.wait_for_service('som/insert_objects')
    rospy.wait_for_service('som/query_object')

    insert_objs_srv = rospy.ServiceProxy('som/insert_objects', SOMInsertObjs)
    query_object_srv = rospy.ServiceProxy('som/query_object', SOMQueryObject)

    my_first_object = SOMObject()
    my_second_object = SOMObject()

    my_first_object.pose.position = Point(1.0, 2.0, 0.5)
    my_second_object.pose.position = Point(1.5, 2.5, 200.0)

    resp = insert_objs_srv([my_first_object, my_second_object])
    print(resp.db_ids[0])
    returned_object = query_object_srv(resp.db_ids[0])

    print(returned_object)

if __name__=="__main__":
    test_database()
