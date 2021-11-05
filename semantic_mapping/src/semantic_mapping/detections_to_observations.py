#!/usr/bin/env python3

import datetime
from orion_actions.msg import *
from orion_actions.srv import *
from geometry_msgs.msg import PoseStamped, Point, Pose
from orion_actions.msg import PoseDetectionPosition
import rospy
import tf2_ros
from orion_actions.msg import DetectionArray, Detection
from tf import TransformListener;
import std_msgs.msg;

# import mongo as MongoInt;
from ebbhrd_msgs.msg import EBBQueryBase, Observation;


class DetectToObserve:
    def __init__(self):
        queue_size = 10;
        
        # Data about transform frames
        self.camera_frame = "head_rgbd_sensor_rgb_frame"
        self.global_frame = "map"

        self.tfBuffer = tf2_ros.Buffer();
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        #Uses tf rather than tf2_ros
        self.tf_old = TransformListener()

        # Initialising the SOM services
        rospy.wait_for_service('som/observe')
        rospy.wait_for_service('som/query')
        self.observe_objs_srv = rospy.ServiceProxy('som/observe', SOMObserve)
        self.query_objs_srv = rospy.ServiceProxy('som/query', SOMQuery)

        # The format of this is:
        # { object_type: forwarding }
        # where forwarding is of the type SOMObservation[] and is what gets sent across som/observe
        # This feels more foolproof than doing a query (as well as potentially being more time efficient) 
        self.previous_detections = {};

        # self.mongodb_client = MongoInt.MongoDBInterface();
        self.observation_pusher = rospy.Publisher('/ebb/observations', Observation, queue_size=queue_size)

        rospy.Subscriber('/vision/bbox_detections', DetectionArray, self.forwardDetectionsToSOM, queue_size=queue_size)
        pass;

    # We essentially want to forward detections from orion_recognition over to the SOM database. This should do that.
    def forwardDetectionsToSOM(self, data:DetectionArray):
    
        for detection in data.detections:
            detection:Detection;
            
            forwarding = SOMObservation();            
            # print(dir(forwarding))
            # print(dir(forwarding.robot_pose));


            # NOTE: Assuming SOMObservation.type is for the name of the object. This is most likely wrong!
            forwarding.type = detection.label.name;
            forwarding.size = detection.size;
            # forwarding.timestamp = rospy.Time().now()

            # Getting the robot pose:
            camera_to_global:tf2_ros.TransformStamped = self.tfBuffer.lookup_transform(self.camera_frame, self.global_frame, rospy.Time());
            # forwarding.robot_pose = Pose();
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
            p_global_frame:PoseStamped = self.tf_old.transformPose(self.global_frame, object_point);
            # transformed_obj_point:PoseStamped = self.tfBuffer.transform(object_point, camera_to_global.transform);
            # transformed_obj_point:PoseStamped = p_global_frame;
            forwarding.pose_observation = p_global_frame.pose;

            forwarding.colour = detection.color;

            # forwarding.header = std_msgs.msg.Header();
            # forwarding.header.stamp = rospy.Time.now();
            # NOTE need to check frame ID in the header. (Could that be that of the camera?)
            # forwarding.header.frame_id = self.global_frame;#.encode("ascii", "ignore");

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
                
            # print(type (forwarding));
            
            service_output:SOMObserveResponse = self.observe_objs_srv(forwarding);    
            addition_successful = service_output.result;
            obj_id_returned = service_output.obj_id;

            if (addition_successful and not item_previously_identified):
                forwarding.obj_id = obj_id_returned;
                # Append to the current array in the dictionary
                if (detection.label.name in self.previous_detections):
                    self.previous_detections[detection.label.name].append(forwarding);
                # Create a new entry
                else:
                    self.previous_detections[detection.label.name] = [forwarding];

            observation_topic_content:Observation = Observation();
            observation_topic_content.base_info.header = "Observation from orion_recognition";
            observation_topic_content.base_info.ros_timestamp = rospy.Time.now();
            observation_topic_content.base_info.location = forwarding.robot_pose;
            observation_topic_content.base_info.entry_type = EBBQueryBase.OBSERVATION;
            observation_topic_content.obj_type = forwarding.type;
            observation_topic_content.obj_colour = forwarding.colour;
            observation_topic_content.location_of_object = forwarding.pose_observation;
            observation_topic_content.id_of_object = obj_id_returned;
            observation_topic_content.obj_previously_identified = item_previously_identified;

            self.observation_pusher.publish(observation_topic_content);
            # self.mongodb_client.addObject(MongoInt.OBSERVATION_COLL, adding_to_mongo);

            print(
                "Obj ID:",  obj_id_returned,
                "\tAdding to SOM:", detection.label.name, 
                "\tSuccessful: ", addition_successful,                
                "\tObject previously identified:", item_previously_identified,                
                "\tposition: ({0:.3f}, {1:.3f}, {1:.3f})".format(
                    forwarding.pose_observation.position.x, 
                    forwarding.pose_observation.position.y, 
                    forwarding.pose_observation.position.z));

        print("------------------")
        pass;

if __name__ == '__main__':
    rospy.init_node('detections_to_observations')
    DetectToObserve();
    rospy.spin();
