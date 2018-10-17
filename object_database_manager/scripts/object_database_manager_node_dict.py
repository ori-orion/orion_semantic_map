#! /usr/bin/env python

import rospy
from object_database_manager.srv import ObjectLocation, ObjectLocationResponse
from strands_navigation_msgs.msg import TopologicalMap
#from orion_object_recognition.msg import ObjectDetection, ObjectDetections
from std_srvs.srv import EmptyResponse
from std_msgs.msg import String
from tmc_vision_msgs.msg import DetectionArray


class ObjectDatabaseManager(object):
    def __init__(self):
        self.numNodes = 0
        self.waypoints = []
        self.currentNode = ""
        self.db = {}

        self.object_query_srv = rospy.Service('/object_query_server', ObjectLocation, self.object_query_cb)
        rospy.loginfo('Ready to be queried for an object location!')

        #self.object_recognition_sub = rospy.Subscriber('/yolo2_object_position_node/result', ObjectDetections, self.object_insertion_cb)
        #rospy.loginfo('Listening on topic /yolo2_object_position_node/result now for recognized objects.')

	self.object_recognition_sub = rospy.Subscriber('/yolo2_node/detections', DetectionArray, self.object_insertion_cb)
        rospy.loginfo('Listening on topic /yolo2_node/detections now for recognized objects.')



        self.top_map_sub = rospy.Subscriber('/topological_map', TopologicalMap, self.update_nodes_cb)
        self.current_node_sub = rospy.Subscriber('/current_node', String, self.update_current_node_cb)

    def update_nodes_cb(self, msg):
        waypoints = [] 
        for node in msg.nodes:
            waypoints.append(str(node.name))
        if self.waypoints == []:
            self.waypoints = waypoints
            rospy.loginfo("The following waypoints are being used: " + ', '.join(waypoints))
            self.numNodes = len(msg.nodes)

    def update_current_node_cb(self, msg):
        if self.currentNode != msg.data:
            rospy.loginfo('CurrentNode has changed from %s to %s!' % (self.currentNode, msg.data))
            self.currentNode = msg.data

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
    
    def object_insertion_cb(self, msg):
        self.db = rospy.get_param("object_locations", {})
 
        for obj in msg.detections:                                                                                                       
            obj_name = obj.label.name.split("-")[0]
            
            if obj_name in self.db:
                self.db[obj_name][self.currentNode] = True 

            else:
                sightings = {}

                for waypoint in rospy.get_param("waypoints").values():
		    #print waypoint                    
		    sightings[waypoint] = False

                sightings[self.currentNode] = True 
		
		print self.currentNode
		#print type(obj_name)
                self.db[obj_name] = sightings
                #rospy.loginfo('Object: "%s" was inserted into the database at waypoint "%s".' % (obj_name, request.waypoint))
	
        rospy.set_param("object_locations", self.db)

    def main(self):
        # Run the program until ctrl-c is sent
        rospy.spin()

if __name__ == "__main__":
    rospy.init_node('object_database_manager')
    manager = ObjectDatabaseManager()
    manager.main()
