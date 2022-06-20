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

import utils;

import numpy;

import tf2_geometry_msgs


class DetectToObserve:
    def __init__(self):
        queue_size = 10;
        
        # Data about transform frames
        self.camera_frame = "head_rgbd_sensor_rgb_frame"
        self.global_frame = "map"

        self.tfBuffer = tf2_ros.Buffer();
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        #Uses tf rather than tf2_ros
        self.tf_old = TransformListener();

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

            if detection.label.confidence < 0.8:
                continue;
            
            forwarding = SOMObservation();            
            # print(dir(forwarding))
            # print(dir(forwarding.robot_pose));


            # NOTE: Assuming SOMObservation.type is for the name of the object. This is most likely wrong!
            if "/" in detection.label.name:
                split_name = detection.label.name.split('/');
                forwarding.class_ = split_name[len(split_name)-1];
                forwarding.category = split_name[len(split_name)-2];
            else:
                forwarding.class_ = detection.label.name;
            forwarding.observation_batch_num = self.batch_num;
            forwarding.size = detection.size;
            # forwarding.timestamp = rospy.Time().now()

            # Getting the robot pose:
            camera_to_global:tf2_ros.TransformStamped = self.tfBuffer.lookup_transform(
                self.camera_frame, 
                self.global_frame, 
                rospy.Time());
            
            # forwarding.robot_pose = Pose();
            forwarding.camera_pose.position.x = camera_to_global.transform.translation.x;
            forwarding.camera_pose.position.y = camera_to_global.transform.translation.y;
            forwarding.camera_pose.position.z = camera_to_global.transform.translation.z;
            forwarding.camera_pose.orientation.w = camera_to_global.transform.rotation.w;
            forwarding.camera_pose.orientation.x = camera_to_global.transform.rotation.x;
            forwarding.camera_pose.orientation.y = camera_to_global.transform.rotation.y;
            forwarding.camera_pose.orientation.z = camera_to_global.transform.rotation.z;

            # Getting the position of the object in 3D space relative to the global frame.
            obj_point_2 = tf2_geometry_msgs.PoseStamped();
            obj_point_2.header.frame_id = self.camera_frame;
            obj_point_2.header.stamp = detection.timestamp;
            obj_point_2.pose.position = Point(
                detection.translation_x, detection.translation_y, detection.translation_z);
            
            # object_point = PoseStamped()
            # object_point.header.frame_id = self.camera_frame;
            # object_point.header.stamp = detection.timestamp;
            # object_point.pose.position = Point(
            #     detection.translation_x, detection.translation_y, detection.translation_z);
            # transformed_stamped = self.tfBuffer.transform(
            #     object_point, self.global_frame, timeout=rospy.Duration(1));
            
            # p_global_frame:PoseStamped = self.tf_old.transformPose(
            #     self.global_frame, object_point);
            p_global_frame:tf2_geometry_msgs.PoseStamped = self.tfBuffer.transform(
                self.global_frame, obj_point_2);
            # transformed_obj_point:PoseStamped = p_global_frame;
            forwarding.obj_position = p_global_frame.pose;

            forwarding.colour = detection.color;

            # forwarding.header = std_msgs.msg.Header();
            # forwarding.header.stamp = rospy.Time.now();
            # NOTE need to check frame ID in the header. (Could that be that of the camera?)
            # forwarding.header.frame_id = self.global_frame;#.encode("ascii", "ignore");                

            # Setting up the covariance stuff.
            # Note that the covariance matrix is symmetric, so the order doesn't matter.
            # Note also that the parameters here could probably be adjusted!
            uncertainty_rot:numpy.matrix = utils.quaternion_to_rot_mat(camera_to_global.transform.rotation);
            cov_mat = numpy.matrix([[3,0,0],[0,0.5,0],[0,0,0.5]]);
            cov_transformed = numpy.matmul(uncertainty_rot.transpose(), numpy.matmul(cov_mat, uncertainty_rot));
            cov_t_linear = [];
            for i in range(3):
                for j in range(3):
                    cov_t_linear.append(cov_transformed[i,j]);
            forwarding.covariance_mat = cov_t_linear;
            
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
