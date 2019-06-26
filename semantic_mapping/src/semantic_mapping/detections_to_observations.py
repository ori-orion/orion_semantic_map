#!/usr/bin/env python
from orion_actions.msg import *
from orion_actions.srv import *
from tf import TransformListener
from geometry_msgs.msg import PoseStamped, Point
import rospy

class DetectToObserve(object):
    def __init__(self):
        self.camera_frame = "/camera_frame_name"
        self.global_frame = "/map"
        rospy.wait_for_service('som/observe')
        rospy.wait_for_service('som/query')
        observe_objs_srv = rospy.ServiceProxy('som/observe', SOMObserve)
        query_objs_srv = rospy.ServiceProxy('som/query', SOMQuery)
        self.tf = TransformListener()
        self.detections_sub = rospy.Subscriber("/vision/bbox_detections", DetectionArray, self.detection_to_observation, queue_size = 5)

    def detection_to_observation(self, data):
        detections = data.detections

        for detection in detections:
            if self.tf.frameExists(self.camera_frame) and self.tf.frameExists(self.global_frame):
                p_camera = PoseStamped()
                p_camera.header.frame_id = self.camera_frame
                p_camera.pose.position = Point(detection.translation_x, detection.translation_y, detection.translation_z)
                p_global = self.tf.transformPose(self.global_frame, p_camera)
                obs.pose_observation = p_global.pose
            else:
                print("ERROR converting detection to observation: both frames %s and %s do not exist" % (self.camera_frame, self.global_frame))

            obs = SOMObservation()
            obs.type = detection.label.name
            obs.colour = detection.color
            obs.size = detection.size

            # if we have observed same object type of same colour before use same object id
            resp = query_object_srv(SOMObservation(type = obs.type, colour = obs.colour), Relation(), SOMObservation(), Pose())
            if resp.matches[0].obj1.observed:
                obs.obj_id = resp.matches[0].obj1.obj_id

            result = observe_objs_srv(obs)
            if not result.result:
                print("Failed to convert detection to observation.")

if __name__ == '__main__':
    rospy.init_node('detections_to_observations')
    DetectToObserve()
