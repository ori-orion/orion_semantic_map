#!/usr/bin/env python3
from orion_actions.msg import *
from orion_actions.srv import *
from geometry_msgs.msg import PoseStamped, Point    #, Pose
from orion_actions.msg import PoseDetectionPosition
import rospy
import tf2_ros
from orion_actions.msg import DetectionArray, Detection

# from cv_bridge import CvBridge, CvBridgeError
# import geometry_msgs.msg
# import message_filters
# import numpy as np
# from sensor_msgs.msg import CameraInfo
# from sensor_msgs.msg import Image
# from tf import TransformListener


class DetectToObserve:
    def __init__(self):
        queue_size = 10;
        
        # Data about transform frames
        self.camera_frame = "head_rgbd_sensor_rgb_frame"
        self.global_frame = "map"

        self.tfBuffer = tf2_ros.Buffer();
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        # Initialising the SOM services
        rospy.wait_for_service('som/observe')
        rospy.wait_for_service('som/query')
        self.observe_objs_srv = rospy.ServiceProxy('som/observe', SOMObserve)
        self.query_objs_srv = rospy.ServiceProxy('som/query', SOMQuery)

        # The format of this is:
        # { object_type: forwarding }
        # where forwarding is of the type SOMObservation and is what gets sent across som/observe
        # This feels more foolproof than doing a query (as well as potentially being more time efficient) 
        self.previous_detections = {};

        rospy.Subscriber('/vision/bbox_detections', DetectionArray, self.forwardDetectionsToSOM, queue_size=queue_size)
        pass;

    # We essentially want to forward detections from orion_recognition over to the SOM database. This should do that.
    def forwardDetectionsToSOM(self, data:DetectionArray):
    
        for detection in data.detections:
            detection:Detection;
            
            forwarding = SOMObservation;

            # NOTE: Assuming SOMObservation.type is for the name of the object. This is most likely wrong!
            forwarding.type = detection.label.name;
            forwarding.size = detection.size;

            # Getting the robot pose:
            camera_to_global:tf2_ros.TransformStamped = self.tfBuffer.lookup_transform(self.camera_frame, self.global_frame, rospy.Time());
            forwarding.robot_pose.position.x = camera_to_global.transform.translation.x;
            forwarding.robot_pose.position.y = camera_to_global.transform.translation.y;
            forwarding.robot_pose.position.z = camera_to_global.transform.translation.z;
            forwarding.robot_pose.orientation.w = camera_to_global.transform.rotation.w;
            forwarding.robot_pose.orientation.x = camera_to_global.transform.rotation.x;
            forwarding.robot_pose.orientation.y = camera_to_global.transform.rotation.y;
            forwarding.robot_pose.orientation.z = camera_to_global.transform.rotation.z;

            # Getting the position of the object in 3D space relative to the global frame.
            object_point = PoseStamped()
            object_point.header.frame_id = self.camera_frame
            object_point.pose.position = Point(detection.translation_x, detection.translation_y, detection.translation_z);
            transformed_obj_point:PoseStamped = self.tfBuffer.transform(forwarding.robot_pose, object_point);
            forwarding.pose_observation = transformed_obj_point.pose;

            forwarding.colour = detection.color;

            forwarding.header.stamp = rospy.Time.now();
            # NOTE need to check frame ID in the header. (Could that be that of the camera?)
            forwarding.header.frame_id = self.camera_frame;

            # We now want to check for duplicates.
            # (Anything that goes through the vision system will be subject to duplicates/multiple).
            item_previously_identified = False;
            if (detection.label.name in self.previous_detections):
                # Check coordinates in size interval. (Essentially we want to check whether 
                # the point is in the bounding box of a previously identified object.)
                def CCISI(prev_coord, current_coord, interval):
                    return -interval/2 < prev_coord - current_coord and prev_coord - current_coord < interval/2
                
                entries = self.previous_detections[detection.label.name];
                
                for entry in entries:
                    entry:SOMObservation;
                    # If this is the case then the current point is in the bounding box of the previous. 
                    # They are therefore one and the same.
                    if (CCISI(forwarding.pose_observation.position.x, entry.pose_observation.position.x, forwarding.size.x) and
                        CCISI(forwarding.pose_observation.position.y, entry.pose_observation.position.y, forwarding.size.y) and 
                        CCISI(forwarding.pose_observation.position.z, entry.pose_observation.position.z, forwarding.size.z)):

                        item_previously_identified = True;
                        forwarding.obj_id = entry.obj_id;
                        break;
                    pass;
                
                pass;    
                
            
            result, obj_id_returned = self.observe_objs_srv(forwarding);    

            if (not item_previously_identified):
                forwarding.obj_id = obj_id_returned;
                self.previous_detections[detection.label.name] = [forwarding];


            print(result);


        pass;

if __name__ == '__main__':
    rospy.init_node('detections_to_observations')
    DetectToObserve()
