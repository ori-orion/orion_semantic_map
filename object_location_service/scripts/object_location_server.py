#! /usr/bin/env python

import rospy
from object_location_service.srv import ObjectLocation, ObjectLocationResponse, InsertObject
from object_location_service.msg import ObjectLocationProbability
from strands_navigation_msgs.msg import TopologicalMap
from mongodb_store.message_store import MessageStoreProxy
from std_srvs.srv import EmptyResponse
from std_msgs.msg import Float32, String

def updateNodes(msg):
    global numNodes
    if numNodes != len(msg.nodes):
        rospy.loginfo('Number of nodes in the Topological Map was updated from %d to %d!' % (numNodes, len(msg.nodes)))
        numNodes = len(msg.nodes)

def handle_object_query(request):
    msg_store = MessageStoreProxy()

    try:
        obj = msg_store.query(ObjectLocationProbability._type, {"object": request.object})
    
        response = ObjectLocationResponse()
        response.probabilities = obj[0][0].probabilities

        return response

    except IndexError, e:
        rospy.loginfo("Object '%s' was most likely not in the database: %s" % (request.object, e))

def handle_object_insertion(request):
    msg_store = MessageStoreProxy() 

    prob = [(0.2 / (numNodes - 1)) for i in range(numNodes)]
    prob[int(request.waypoint)] = 0.8

    obj = ObjectLocationProbability(request.object, prob) 

    msg_store.insert(obj)

    rospy.loginfo('Object: "%s" was inserted into the database.' % obj.object)
    
    response = EmptyResponse()
    return response

def object_location_server():
    rospy.init_node('object_location_server')
   
    object_query_srv = rospy.Service('/object_query_server', ObjectLocation, handle_object_query)
    rospy.loginfo('Ready to be queried for an object location!')

    object_insertion_srv = rospy.Service('/object_insertion_server', InsertObject, handle_object_insertion) 
    rospy.loginfo('Ready for objects to be inserted into the database!') 
    sub = rospy.Subscriber('/topological_map', TopologicalMap, updateNodes)
    
    rospy.spin()

if __name__ == "__main__":
    numNodes = 0 
    object_location_server()
