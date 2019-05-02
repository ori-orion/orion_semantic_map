#!/usr/bin/env python

import roslib, rospy, json, argparse, random, os
import copy, sys, datetime, time, math

from geometry_msgs.msg import Pose, Point
from geometry_msgs.msg import PoseArray
from threading import Timer
from mongodb_store.message_store import MessageStoreProxy
from semantic_mapping.msg import SOMObservation, SOMObject
from semantic_mapping.srv import *
from std_msgs.msg import String

def test_database():
    rospy.wait_for_service('som/observe')
    rospy.wait_for_service('som/lookup')
    rospy.wait_for_service('som/query')

    observe_objs_srv = rospy.ServiceProxy('som/observe', SOMObserve)
    lookup_object_srv = rospy.ServiceProxy('som/lookup', SOMLookup)
    delete_object_srv = rospy.ServiceProxy('som/delete', SOMDelete)
    query_object_srv = rospy.ServiceProxy('som/query', SOMQuery)

    my_first_observation = SOMObservation()
    my_second_observation = SOMObservation()

    my_first_observation.pose_observation.position = Point(1.0, 2.0, 0.5)
    my_first_observation.type = "shirt"
    my_first_observation.colour = "red"

    my_second_observation.pose_observation.position = Point(1.5, 2.5, 200.0)
    my_second_observation.type = "shirt"
    my_second_observation.colour = "blue"

    mydir = os.path.dirname(__file__)
    robocup_onto_path = os.path.join(mydir, '../config/robocupontology.owl')

    resp = observe_objs_srv([my_first_observation, my_second_observation])

    resp = query_object_srv(my_first_observation, 10, SOMObservation(), Pose())
    print(resp)
    #print(resp)
    #returned_object = lookup_object_srv(resp.obj_ids[0])
    #print(returned_object)



if __name__=="__main__":
    test_database()
