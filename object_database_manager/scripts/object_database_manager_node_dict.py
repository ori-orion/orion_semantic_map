#! /usr/bin/env python

import rospy
from object_database_manager.srv import ObjectLocation, ObjectLocationResponse
from std_srvs.srv import EmptyResponse
from std_msgs.msg import String
from tmc_vision_msgs.msg import DetectionArray


class ObjectDatabaseManager(object):
    def __init__(self):
        self.num_nodes = 0
        self.current_node = ""
        self.db = rospy.get_param('/object_locations', {})

        self.object_query_srv = rospy.Service('/object_query_server', ObjectLocation, self.object_query_cb)
        rospy.loginfo('Ready to be queried for an object location!')

        #self.object_recognition_sub = rospy.Subscriber('/yolo2_object_position_node/result', ObjectDetections, self.object_insertion_cb)
        #rospy.loginfo('Listening on topic /yolo2_object_position_node/result now for recognized objects.')

	self.object_recognition_sub = rospy.Subscriber('/check_object_server_node/detections', DetectionArray, self.object_insertion_cb)
        rospy.loginfo('Listening on topic /yolo2_node/detections now for recognized objects.')

        self.obj_waypoints = rospy.get_param('/bring_me/object_waypoints')
        self.num_nodes = len(self.obj_waypoints)
        rospy.loginfo("The following waypoints are being used: " + ', '.join(self.obj_waypoints))
        
        # if the database doesn't exist yet fill it with all objects but initialize them as 'not_checked'
        if not self.db:
            for obj in rospy.get_param('/bring_me/recognisability'):
                sightings = {}
                for waypoint in self.obj_waypoints:
                    sightings[waypoint] = -1
                self.db[obj] = sightings
        rospy.set_param("object_locations", self.db)

        self.current_node_sub = rospy.Subscriber('/current_node', String, self.update_current_node_cb)

    def update_current_node_cb(self, msg):
        if self.current_node != msg.data:
            rospy.loginfo('CurrentNode has changed from %s to %s!' % (self.current_node, msg.data))
            self.current_node = msg.data

    def object_query_cb(self, request):
        self.db = rospy.get_param("object_locations", {})
        response = ObjectLocationResponse()
        
        if request.object in self.db:
            # if the object has been seen before (e.g. only at Table2) it is in the db and based on the amount of sightings 
            # we can calculate the probabilities and return them together with the waypoints in the following form 
            # probabilities: [0.800, 0.025, 0.025, 0.025, 0.025, 0.025, 0.025, 0.025, 0.025]
            # waypoints: [Table2, Table1, Waypoint2, Waypoint3, Waypoint1, Shelf2, Shelf1, Fridge, Kitchen] 

            sightings = self.db[request.object]
            amount_seen = sum(value == 1 for value in self.db[request.object].values())
            
            for waypoint, val in sightings.iteritems():
                if val == 1:
                    response.probabilities.append(0.8)
                    response.waypoints.append(waypoint)
                elif val == -1 and amount_seen > 0:
                    response.probabilities.append(0.3)
                    response.waypoints.append(waypoint)
                elif val == -1 and amount_seen == 0:
                    response.probabilities.append(0.5)
                    response.waypoints.append(waypoint)
                elif val == 0:
                    response.probabilities.append(0.1)
                    response.waypoints.append(waypoint)
        else:
            # because the object is not in the db yet just spread the probability evenly over all waypoints

            response.probabilities = [(1.0 / self.num_nodes) for i in range(self.num_nodes)]
            for waypoint in self.obj_waypoints:
                response.waypoints.append(waypoint)
        
        return response
    
    def object_insertion_cb(self, msg):
        self.db = rospy.get_param("object_locations", {})
        
        if self.current_node in self.obj_waypoints:
            seen_objs = []
            for obj in msg.detections:
                seen_objs.append(obj.label.name.split("-")[0])
                
            for obj in self.db:
                if obj in seen_objs:
                    self.db[obj][self.current_node] = 1
                    rospy.loginfo("%s has been seen at waypoint: %s" % (obj, self.current_node))
                if self.db[obj][self.current_node] == -1:
                    self.db[obj][self.current_node] = 0

            rospy.set_param("object_locations", self.db)

    def main(self):
        # Run the program until ctrl-c is sent
        rospy.spin()

if __name__ == "__main__":
    rospy.init_node('object_database_manager')
    manager = ObjectDatabaseManager()
    manager.main()
