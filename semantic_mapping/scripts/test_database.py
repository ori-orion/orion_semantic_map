#!/usr/bin/env python

import roslib, rospy, json, argparse, random, os, pronto
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

    observe_objs_srv = rospy.ServiceProxy('som/observe', SOMObserve)
    lookup_object_srv = rospy.ServiceProxy('som/lookup', SOMLookup)
    delete_object_srv = rospy.ServiceProxy('som/lookup', SOMLookup)

    my_first_observation = SOMObservation()
    my_second_observation = SOMObservation()
    print(my_first_observation.obj_id)
    my_first_observation.pose_observation.position = Point(1.0, 2.0, 0.5)
    my_second_observation.pose_observation.position = Point(1.5, 2.5, 200.0)

    mydir = os.path.dirname(__file__)
    robocup_onto_path = os.path.join(mydir, '../config/robocupontology.owl')

    resp = observe_objs_srv([my_first_observation, my_second_observation])
    print(resp)
    returned_object = lookup_object_srv(resp.obj_ids[0])
    print(returned_object)

    delete_object_srv(resp.obj_ids[0])
    returned_object = lookup_object_srv(resp.obj_ids[0])
    print(returned_object)

if __name__=="__main__":
    test_database()
