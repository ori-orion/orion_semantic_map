#!/usr/bin/env python3
from orion_actions.msg import *
from orion_actions.srv import *
from tf import TransformListener
from geometry_msgs.msg import PoseStamped, Point
from orion_actions.msg import PoseDetectionPosition
from tmc_vision_msgs.msg import DetectionLocation
import rospy

class DetectToObserve(object):
    def __init__(self):
        self.camera_frame = "head_rgbd_sensor_rgb_frame"
        self.global_frame = "map"
        rospy.wait_for_service('som/observe')
        rospy.wait_for_service('som/query')
        observe_objs_srv = rospy.ServiceProxy('som/observe', SOMObserve)
        query_objs_srv = rospy.ServiceProxy('som/query', SOMQuery)
        self.tf = TransformListener()
        self.detections_sub = rospy.Subscriber("/vision/detection_locations", DetectionLocation, self.detection_to_observation, queue_size = 5)
        self.detections_sub = rospy.Subscriber("/vision/pose_locations", PoseDetectionPosition, self.person_observations, queue_size = 5)

    def detection_to_observation(self, data):
        obs = SOMObservation()
        if self.tf.frameExists(self.camera_frame) and self.tf.frameExists(self.global_frame):
            p_camera = PoseStamped()
            p_camera.header.frame_id = self.camera_frame
            p_camera.pose.position = Point(data.x_cam_m, data.y_cam_m, data.z_cam_m)
            p_global = self.tf.transformPose(self.global_frame, p_camera)
            obs.pose_observation = p_global.pose
        else:
            print("ERROR converting detection to observation: both frames %s and %s do not exist" % (self.camera_frame, self.global_frame))
        obs.type = data.label.name
        obs.size = Point(data.bb_width_m, data.bb_height_m, data.bb_depth_m)

        # if we have observed same object type of same colour before use same object id
        resp = query_object_srv(SOMObservation(type = obs.type, colour = obs.colour), Relation(), SOMObservation(), Pose())
        if resp.matches[0].obj1.observed:
            obs.obj_id = resp.matches[0].obj1.obj_id

        result = observe_objs_srv(obs)
        if not result.result:
            print("Failed to convert detection to observation.")

    def person_observations(self, data):
        obs = SOMObservation()
        if self.tf.frameExists(self.camera_frame) and self.tf.frameExists(self.global_frame):
            p_camera = PoseStamped()
            p_camera.header.frame_id = self.camera_frame
            p_camera.pose.position = Point(data.x_cam_m, data.y_cam_m, data.z_cam_m)
            p_global = self.tf.transformPose(self.global_frame, p_camera)
            obs.pose_observation = p_global.pose
        else:
            print("ERROR converting detection to observation: both frames %s and %s do not exist" % (self.camera_frame, self.global_frame))
        obs.colour = data.color
        obs.type = 'person'

        resp = query_object_srv(SOMObservation(type = obs.type, colour = obs.colour), Relation(), SOMObservation(), Pose())
        if resp.matches[0].obj1.observed:
            obs.obj_id = resp.matches[0].obj1.obj_id

        result = observe_objs_srv(obs)
        if not result.result:
            print("Failed to convert detection to observation.")

if __name__ == '__main__':
    rospy.init_node('detections_to_observations')
    DetectToObserve()
