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
        self.waypoints = []
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
        waypoints = [] 
        for node in msg.nodes:
            waypoints.append(str(node.name))
        if self.waypoints == []:
            self.waypoints = waypoints
            rospy.loginfo("The following waypoints are being used: " + ', '.join(waypoints))
            self.numNodes = len(msg.nodes)

    def update_current_node_cb(self, msg):
        if self.currentNode != msg:
            rospy.loginfo('CurrentNode has changed from %s to %s!' % (self.currentNode, msg))
            # self.currentNode = msg

    def object_query_cb(self, request):
        self.db = rospy.get_param("object_locations", {})
        response = ObjectLocationResponse()
        
        if request.object in self.db:
            # if the object has been seen before (e.g. only at Table2) it is in the db and based on the amount of sightings 
            # we can calculate the probabilities and return them together with the waypoints in the following form 
            # probabilities: [0.800, 0.025, 0.025, 0.025, 0.025, 0.025, 0.025, 0.025, 0.025]
            # waypoints: [Table2, Table1, Waypoint2, Waypoint3, Waypoint1, Shelf2, Shelf1, Fridge, Kitchen] 

            sightings = self.db[request.object]
            amountTrue = sum(value == True for value in self.db[request.object].values())

            for waypoint, val in sightings.iteritems():
                if val:
                    response.probabilities.append(0.8 / amountTrue)
                    response.waypoints.append(waypoint)
                else:
                    response.probabilities.append(0.2 / (self.numNodes - amountTrue))
                    response.waypoints.append(waypoint)
        else:
            # because the object is not in the db yet just spread the probability evenly over all waypoints

            response.probabilities = [(1.0 / self.numNodes) for i in range(self.numNodes)]
            for waypoint in rospy.get_param("waypoints").values():
                response.waypoints.append(waypoint)
        
        return response
    
    def object_insertion_cb(self, request):
        self.currentNode = request.waypoint # for testing
        self.db = rospy.get_param("object_locations", {})

        if request.object in self.db:
            self.db[request.object][self.currentNode] = True 
            rospy.set_param("object_locations", self.db)

            return EmptyResponse()
        else:
            sightings = {}

            for waypoint in rospy.get_param("waypoints").values():
                sightings[waypoint] = False

            sightings[self.currentNode] = True 

            self.db[request.object] = sightings
           
            rospy.set_param("object_locations", self.db)

            rospy.loginfo('Object: "%s" was inserted into the database at waypoint "%s".' % (request.object, request.waypoint))
            
            return EmptyResponse()

    def main(self):
        # Run the program until ctrl-c is sent
        rospy.spin()

if __name__ == "__main__":
    rospy.init_node('object_database_manager')
    manager = ObjectDatabaseManager()
    manager.main()
