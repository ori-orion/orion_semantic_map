#!/usr/bin/env python3

from orion_actions.msg import *
from orion_actions.srv import *
from geometry_msgs.msg import PoseStamped, Point, Pose
from orion_actions.msg import PoseDetectionPosition
import rospy
import tf2_ros
from orion_actions.msg import DetectionArray, Detection
from tf import TransformListener;
import std_msgs.msg;


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
        # rospy.wait_for_service('som/observe')
        # rospy.wait_for_service('som/query')
        rospy.wait_for_service('som/observations/input');
        self.observe_obj_srv = rospy.ServiceProxy(
            'som/observations/input', 
            SOMAddObservation);
        # self.observe_objs_srv = rospy.ServiceProxy('som/observe', SOMObserve)
        # self.query_objs_srv = rospy.ServiceProxy('som/query', SOMQuery)

        # The format of this is:
        # { object_type: forwarding }
        # where forwarding is of the type SOMObservation[] and is what gets sent across som/observe
        # This feels more foolproof than doing a query (as well as potentially being more time efficient) 
        # self.previous_detections = {};

        rospy.Subscriber('/vision/bbox_detections', DetectionArray, self.forwardDetectionsToSOM, queue_size=queue_size)

        self.batch_num = 1;        
        pass;

    # We essentially want to forward detections from orion_recognition over to the SOM database. This should do that.
    def forwardDetectionsToSOM(self, data:DetectionArray):
    
        for detection in data.detections:
            detection:Detection;
            
            forwarding = SOMObservation();            
            # print(dir(forwarding))
            # print(dir(forwarding.robot_pose));


            # NOTE: Assuming SOMObservation.type is for the name of the object. This is most likely wrong!
            forwarding.class_ = detection.label.name;
            forwarding.observation_batch_num = self.batch_num;
            forwarding.size = detection.size;
            # forwarding.timestamp = rospy.Time().now()

            # Getting the robot pose:
            camera_to_global:tf2_ros.TransformStamped = self.tfBuffer.lookup_transform(self.camera_frame, self.global_frame, rospy.Time());
            # forwarding.robot_pose = Pose();
            forwarding.camera_pose.position.x = camera_to_global.transform.translation.x;
            forwarding.camera_pose.position.y = camera_to_global.transform.translation.y;
            forwarding.camera_pose.position.z = camera_to_global.transform.translation.z;
            forwarding.camera_pose.orientation.w = camera_to_global.transform.rotation.w;
            forwarding.camera_pose.orientation.x = camera_to_global.transform.rotation.x;
            forwarding.camera_pose.orientation.y = camera_to_global.transform.rotation.y;
            forwarding.camera_pose.orientation.z = camera_to_global.transform.rotation.z;

            # Getting the position of the object in 3D space relative to the global frame.
            object_point = PoseStamped()
            object_point.header.frame_id = self.camera_frame
            object_point.pose.position = Point(detection.translation_x, detection.translation_y, detection.translation_z);            
            p_global_frame:PoseStamped = self.tf_old.transformPose(self.global_frame, object_point);
            # transformed_obj_point:PoseStamped = self.tfBuffer.transform(object_point, camera_to_global.transform);
            # transformed_obj_point:PoseStamped = p_global_frame;
            forwarding.obj_position = p_global_frame.pose;

            forwarding.colour = detection.color;

            # forwarding.header = std_msgs.msg.Header();
            # forwarding.header.stamp = rospy.Time.now();
            # NOTE need to check frame ID in the header. (Could that be that of the camera?)
            # forwarding.header.frame_id = self.global_frame;#.encode("ascii", "ignore");                
        
            
            service_output:SOMObserveResponse = self.observe_obj_srv(forwarding);
            print(forwarding.class_);
            # addition_successful = service_output.obj_id;
            # obj_id_returned = service_output.obj_id;

            # print(obj_id_returned);
        print("--------------------------------");
        self.batch_num += 1;


if __name__ == '__main__':
    rospy.init_node('detections_to_observations')
    DetectToObserve();
    rospy.spin();
