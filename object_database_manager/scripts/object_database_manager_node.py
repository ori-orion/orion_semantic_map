#! /usr/bin/env python

import rospy
from object_database_manager.srv import ObjectLocation, ObjectLocationResponse, InsertObject
from object_database_manager.msg import ObjectLocationProbability
from strands_navigation_msgs.msg import TopologicalMap
from mongodb_store.message_store import MessageStoreProxy
from std_srvs.srv import EmptyResponse
from std_msgs.msg import Float32, String


class ObjectDatabaseManager(object):
    def __init__(self):
        self.numNodes = 0

        self.object_query_srv = rospy.Service('/object_query_server', ObjectLocation, self.object_query_cb)
        rospy.loginfo('Ready to be queried for an object location!')

        self.object_insertion_srv = rospy.Service('/object_insertion_server', InsertObject, self.object_insertion_cb)
        rospy.loginfo('Ready for objects to be inserted into the database!')

        self.top_map_sub = rospy.Subscriber('/topological_map', TopologicalMap, self.updateNodes)
        
        self.msg_store = MessageStoreProxy()

    def updateNodes(self, msg):
        if self.numNodes != len(msg.nodes):
            rospy.loginfo('Number of nodes in the Topological Map was updated from %d to %d!' % (self.numNodes, len(msg.nodes)))
            self.numNodes = len(msg.nodes)

    def object_query_cb(self, request):
        try:
            obj = self.msg_store.query(ObjectLocationProbability._type, {"object": request.object})
        
            response = ObjectLocationResponse()
            
            response.probabilities = obj[0][0].probabilities
    
            return response
    
        except IndexError, e:
            rospy.loginfo("Object '%s' was most likely not in the database: %s" % (request.object, e))

    def object_insertion_cb(self, request):
        msg_store = MessageStoreProxy()

        prob = [(0.2 / (self.numNodes - 1)) for i in range(self.numNodes)]
        prob[int(request.waypoint)] = 0.8
    
        obj = ObjectLocationProbability(request.object, prob) 
    
        self.msg_store.insert(obj)
    
        rospy.loginfo('Object: "%s" was inserted into the database at waypoint "%s".' % (obj.object, request.waypoint))
        
        response = EmptyResponse()
        return response

    def main(self):
        # Run the program until ctrl-c is sent
        rospy.spin()

if __name__ == "__main__":
    rospy.init_node('object_database_manager')
    manager = ObjectDatabaseManager()
    manager.main()
