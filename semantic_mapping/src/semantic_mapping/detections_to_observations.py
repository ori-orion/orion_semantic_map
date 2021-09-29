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


# class DetectionTFPublisher(object):
#     def __init__(self):
#         self.camera_frame = "head_rgbd_sensor_rgb_frame"
#         self.global_frame = "map"
#         rospy.wait_for_service('som/observe')
#         rospy.wait_for_service('som/query')
#         self.observe_objs_srv = rospy.ServiceProxy('som/observe', SOMObserve)
#         self.query_objs_srv = rospy.ServiceProxy('som/query', SOMQuery)
        
#         self.tfBuffer = tf2_ros.Buffer();
#         self.listener = tf2_ros.TransformListener(self.tfBuffer)

#         self.bridge = CvBridge()
#         detection_sub = message_filters.Subscriber(
#             "/vision/bbox_detections", DetectionArray)
#         depth_sub = message_filters.Subscriber(
#             "/hsrb/head_rgbd_sensor/depth_registered/image_rect_raw", Image)
#         self._objects = rospy.get_param('~objects', [])
#         self._ts = message_filters.ApproximateTimeSynchronizer(
#             [detection_sub, depth_sub], 30, 0.5)
#         self._ts.registerCallback(self.callback)
#         # self._br = tf2_ros.TransformBroadcaster()



#     def callback(self, detections, depth_data):
#         objects = {key.label.name: [] for key in detections.detections} #self._objects}
#         trans = []
#         for detection in detections.detections:
#                 object_point = [detection.translation_x, detection.translation_y, detection.translation_z]
#                 objects[detection.label.name].append(object_point)
#         print(objects)
        
#         for obj in objects:
#             for i, pos in enumerate(objects[obj]):
#                 t = geometry_msgs.msg.TransformStamped()
#                 t.header = depth_data.header
#                 t.child_frame_id = obj + '_' + str(i)
#                 t.transform.translation.x = pos[0]
#                 t.transform.translation.y = pos[1]
#                 t.transform.translation.z = pos[2]
#                 # compute the tf frame
#                 # rotate -90 degrees along z-axis
#                 t.transform.rotation.z = np.sin(-np.pi / 4)
#                 t.transform.rotation.w = np.cos(-np.pi / 4)
#                 trans.append(t)

#             observation = SOMObservation();

#             try:
#                 camera_to_global = self.tfBuffer.lookup_transform(self.camera_frame, self.global_frame, rospy.Time())
#                 p_camera = PoseStamped();
#                 p_camera.header.frame_id = self.camera_frame                
#                 p_camera.pose.position.x = camera_to_global.transform.translation.x;
#                 p_camera.pose.position.y = camera_to_global.transform.translation.y;
#                 p_camera.pose.position.z = camera_to_global.transform.translation.z;
#                 p_camera.pose.orientation.w = camera_to_global.transform.rotation.w;
#                 p_camera.pose.orientation.x = camera_to_global.transform.rotation.x;
#                 p_camera.pose.orientation.y = camera_to_global.transform.rotation.y;
#                 p_camera.pose.orientation.z = camera_to_global.transform.rotation.z;
#                 p_global = self.tf.transformPose(self.global_frame, p_camera)
#                 observation.pose_observation = p_global.pose
#                 pass;
#             except Exception as ex:
#                 rospy.logerr(ex);

#             if self.tf.frameExists(self.camera_frame) and self.tf.frameExists(self.global_frame):
#                 p_camera = PoseStamped()
#                 p_camera.header.frame_id = self.camera_frame
#                 p_camera.pose.position = Point(data.x_cam_m, data.y_cam_m, data.z_cam_m)
#                 p_global = self.tf.transformPose(self.global_frame, p_camera)
#                 observation.pose_observation = p_global.pose
#             else:
#                 print("ERROR converting detection to observation: both frames %s and %s do not exist" % (self.camera_frame, self.global_frame))

#             serviceReturn = self.observe_objs_srv(observation);
#             print(serviceReturn);
#         # self._br.sendTransform(trans)






# if __name__ == '__main__':
#     rospy.init_node('detection_tf_publisher')
#     DetectionTFPublisher()
#     rospy.spin()


# class DetectToObserve(object):
#     def __init__(self):
#         self.camera_frame = "head_rgbd_sensor_rgb_frame"
#         self.global_frame = "map"
#         rospy.wait_for_service('som/observe')
#         rospy.wait_for_service('som/query')
#         self.observe_objs_srv = rospy.ServiceProxy('som/observe', SOMObserve)
#         self.query_objs_srv = rospy.ServiceProxy('som/query', SOMQuery)
#         self.tf = TransformListener()
#         self.detections_sub = rospy.Subscriber("/vision/detection_locations", DetectionLocation, self.detection_to_observation, queue_size = 5)
#         self.detections_sub = rospy.Subscriber("/vision/pose_locations", PoseDetectionPosition, self.person_observations, queue_size = 5)

#     def detection_to_observation(self, data):
#         obs = SOMObservation()
#         if self.tf.frameExists(self.camera_frame) and self.tf.frameExists(self.global_frame):
#             p_camera = PoseStamped()
#             p_camera.header.frame_id = self.camera_frame
#             p_camera.pose.position = Point(data.x_cam_m, data.y_cam_m, data.z_cam_m)
#             p_global = self.tf.transformPose(self.global_frame, p_camera)
#             obs.pose_observation = p_global.pose
#         else:
#             print("ERROR converting detection to observation: both frames %s and %s do not exist" % (self.camera_frame, self.global_frame))
#         obs.type = data.label.name
#         obs.size = Point(data.bb_width_m, data.bb_height_m, data.bb_depth_m)

#         # if we have observed same object type of same colour before use same object id
#         resp = self.query_objs_srv(SOMObservation(type = obs.type, colour = obs.colour), Relation(), SOMObservation(), Pose())
#         if resp.matches[0].obj1.observed:
#             obs.obj_id = resp.matches[0].obj1.obj_id

#         result = self.observe_objs_srv(obs)
#         if not result.result:
#             print("Failed to convert detection to observation.")

#     def person_observations(self, data):
#         obs = SOMObservation()
#         if self.tf.frameExists(self.camera_frame) and self.tf.frameExists(self.global_frame):
#             p_camera = PoseStamped()
#             p_camera.header.frame_id = self.camera_frame
#             p_camera.pose.position = Point(data.x_cam_m, data.y_cam_m, data.z_cam_m)
#             p_global = self.tf.transformPose(self.global_frame, p_camera)
#             obs.pose_observation = p_global.pose
#         else:
#             print("ERROR converting detection to observation: both frames %s and %s do not exist" % (self.camera_frame, self.global_frame))
#         obs.colour = data.color
#         obs.type = 'person'

#         resp = self.query_objs_srv(SOMObservation(type = obs.type, colour = obs.colour), Relation(), SOMObservation(), Pose())
#         if resp.matches[0].obj1.observed:
#             obs.obj_id = resp.matches[0].obj1.obj_id

#         result = self.observe_objs_srv(obs)
#         if not result.result:
#             print("Failed to convert detection to observation.")

if __name__ == '__main__':
    rospy.init_node('detections_to_observations')
    DetectToObserve()
