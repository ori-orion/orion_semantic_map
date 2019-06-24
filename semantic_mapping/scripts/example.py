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
    rospy.wait_for_service('som/get_all_objects')
    rospy.wait_for_service('som/check_similarity')

    observe_objs_srv = rospy.ServiceProxy('som/observe', SOMObserve)
    lookup_object_srv = rospy.ServiceProxy('som/lookup', SOMLookup)
    delete_object_srv = rospy.ServiceProxy('som/delete', SOMDelete)
    query_object_srv = rospy.ServiceProxy('som/query', SOMQuery)
    clear_database_srv = rospy.ServiceProxy('som/clear_database', SOMClearDatabase)
    get_all_objects_srv = rospy.ServiceProxy('som/get_all_objects', SOMGetAllObjects)
    check_similarity_srv = rospy.ServiceProxy('som/check_similarity', SOMCheckSimilarity)

    # clear out the database
    clear_database_srv()

    # observe some bacon
    my_first_observation = SOMObservation()
    my_first_observation.pose_observation.position = Point(0.2, 0.2, 0.5)
    my_first_observation.size = Point(0.4, 0.5, 0.3)
    my_first_observation.type = "Bacon"
    my_first_observation.colour = "red"
    resp = observe_objs_srv(my_first_observation)
    bacon_id = resp.obj_id

    # observe some pizza
    my_second_observation = SOMObservation()
    my_second_observation.pose_observation.position = Point(0.1, 0.3, 0.3)
    my_second_observation.size = Point(1.0, 0.5, 0.3)
    my_second_observation.type = "Pizza"
    my_second_observation.colour = "blue"
    resp = observe_objs_srv(my_second_observation)
    pizza_id = resp.obj_id

    # observe some milk
    my_third_observation = SOMObservation()
    my_third_observation.pose_observation.position = Point(0.0,  0.5, -0.3)
    my_third_observation.size = Point(1.0, 0.5, 0.3)
    my_third_observation.type = "Milk"
    my_third_observation.colour = "white"
    resp = observe_objs_srv(my_third_observation)
    milk_id = resp.obj_id

    # query for all bacon objects
    resp = query_object_srv(SOMObservation(type = 'bacon'), Relation(), SOMObservation(), Pose())
    print("The number of bacon objects observed is %d\n" % (len(resp.matches)))

    # query for all blue bacon objects
    resp = query_object_srv(SOMObservation(type = 'bacon', colour = 'blue'), Relation(), SOMObservation(), Pose())
    print("The number of blue bacon objects observed is %d\n" % (len(resp.matches)))

    # query for all red bacon objects
    resp = query_object_srv(SOMObservation(type = 'bacon', colour = 'red'), Relation(), SOMObservation(), Pose())
    print("The number of red bacon objects observed is %d\n" % (len(resp.matches)))

    # query for objects above the pizza
    resp = query_object_srv(SOMObservation(), Relation(above=True), SOMObservation(type = 'milk'), Pose())
    print("The number of objects above the milk is %d\n" % (len(resp.matches)))
    print("The objects above the milk are %s and %s\n" % (resp.matches[0].obj1.type, resp.matches[1].obj1.type))

    # query for all foods
    resp = query_object_srv(SOMObservation(type = 'food'), Relation(), SOMObservation(), Pose())
    print("The number of food objects is %d\n" % (len(resp.matches)))

    # update that the bacon has gone off and is now blue
    my_fourth_observation = SOMObservation(obj_id = bacon_id, colour = 'blue')
    observe_objs_srv(my_fourth_observation)
    print("We observed the bacon is now blue")

    # query for all blue bacon objects
    resp = query_object_srv(SOMObservation(type = 'bacon', colour = 'blue'), Relation(), SOMObservation(), Pose())
    print("The number of blue bacon objects observed is %d\n" % (len(resp.matches)))

    # query for all red bacon objects
    resp = query_object_srv(SOMObservation(type = 'bacon', colour = 'red'), Relation(), SOMObservation(), Pose())
    print("The number of red bacon objects observed is %d\n" % (len(resp.matches)))

    # query for the relationship between bacon and milk
    resp = query_object_srv(SOMObservation(obj_id = bacon_id), Relation(), SOMObservation(obj_id = pizza_id), Pose())
    print("The relationship between the bacon and the milk is %s\n" % (resp.matches[0].relation))

    # query for the left_most object which is near the pizza
    resp = query_object_srv(SOMObservation(), Relation(near=True, left_most=True), SOMObservation(obj_id = pizza_id), Pose())
    print("The left most object near the pizza is the %s\n" % (resp.matches[0].obj1.type))

    # query similarity
    resp = check_similarity_srv(SOMObservation(type = 'pizza'), SOMObservation(type = 'bacon'))
    print("The similiary between pizza and bacon is %i\n" % (resp.similarity))

    resp = check_similarity_srv(SOMObservation(type = 'pizza'), SOMObservation(type = 'milk'))
    print("The similiary between pizza and milk is %i\n" % (resp.similarity))

if __name__=="__main__":
    test_database()
