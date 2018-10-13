#! /usr/bin/env python

import rospy
from object_database_manager.srv import ObjectLocation, ObjectLocationResponse, InsertObject
from object_database_manager.msg import ObjectLocationProbability, ObjectLocations
from strands_navigation_msgs.msg import TopologicalMap
from mongodb_store.message_store import MessageStoreProxy
from std_srvs.srv import EmptyResponse
from std_msgs.msg import Float32, String


class ObjectDatabaseManager(object):
    def __init__(self):
        self.numNodes = 0
        self.currentNode = "Waypoint0"
        self.db = {}

        self.object_query_srv = rospy.Service('/object_query_server', ObjectLocation, self.object_query_cb)
        rospy.loginfo('Ready to be queried for an object location!')

        self.object_insertion_srv = rospy.Service('/object_insertion_server', InsertObject, self.object_insertion_cb)
        rospy.loginfo('Ready for objects to be inserted into the database!')

        self.top_map_sub = rospy.Subscriber('/topological_map', TopologicalMap, self.update_nodes_cb)
        self.current_node_sub = rospy.Subscriber('/current_node', String, self.update_current_node_cb)

        self.msg_store = MessageStoreProxy()

    def update_nodes_cb(self, msg):
        if self.numNodes != len(msg.nodes):
            rospy.loginfo('Number of nodes in the Topological Map was updated from %d to %d!' % (self.numNodes, len(msg.nodes)))
            self.numNodes = len(msg.nodes)

    def update_current_node_cb(self, msg):
        if self.currentNode != msg:
            rospy.loginfo('CurrentNode has changed from %s to %s!' % (self.currentNode, msg))
            # self.currentNode = msg

    def object_query_cb(self, request):
        sightings = self.db[request.object]
        amountTrue = sightings.count(True)
        prob = [0] * self.numNodes

        for idx, val in enumerate(sightings):
            if val:
                prob[idx] = 0.8 / amountTrue
            else:
                prob[idx] = 0.2 / (self.numNodes - amountTrue)
        
        response = ObjectLocationResponse()
        response.probabilities = prob
        return response
    
    def object_insertion_cb(self, request):
        self.currentNode = request.waypoint # for testing

        if request.object in self.db:
            self.db[request.object][int(self.currentNode[-1:])] = True
            
            response = EmptyResponse()
            return response
        else:
            sightings = [False for i in range(self.numNodes)]
            sightings[int(self.currentNode[-1:])] = True

            self.db[request.object] = sightings
           
            rospy.loginfo('Object: "%s" was inserted into the database at waypoint "%s".' % (request.object, request.waypoint))
            response = EmptyResponse()
            return response

    def main(self):
        # Run the program until ctrl-c is sent
        rospy.spin()

if __name__ == "__main__":
    rospy.init_node('object_database_manager')
    manager = ObjectDatabaseManager()
    manager.main()
