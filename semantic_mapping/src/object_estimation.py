import message_conversion
import rospy
from semantic_mapping.msg import *

def make_observation(obs, object_store, observation_store):
    ''' Receives an observation and updates the object and observation stores
    accordingly.


    If no obj_id is supplied in the observation, we add a new object, and an
    observation of the object to the database. The pose_estimate of the new
    object is based on the single new observation.

    If an obj_id is supplied, we add the new observation of the object to the
    observation database. We retrieve all observations of the object from the
    database. The pose_estimate of the existing object is updated based on
    all of the observations retrieved.
    '''
    time_now = rospy.Time.now().secs
    obs.timestamp = time_now
    obj = message_conversion.observation_to_object(obs, default_frame_id="/map")

    ## if no object id is supplied insert new object
    if obs.obj_id == "":

        # pose estimate based on single observation
        obj.pose_estimate = estimate_pose([obs])
        try:
            obj_id = object_store.insert(obj)
            obs.obj_id = obj_id
            obs_id = observation_store.insert(obs)

        except rospy.ServiceException, e:
            print("Service call failed: %s"%(e))
            obj_id = ""
            return (False, obj_id)

    ## if the object id is supplied then update existing
    else:
        try:
            # add observation to store and get all observations of object
            obs_id = observation_store.insert(obs)
            obj_id = obs.obj_id
            response = observation_store.query(SOMObservation._type,message_query={"obj_id": obj_id})
            all_observations = [i[0] for i in response]

            # update object with new pose estimate
            obj.pose_estimate = estimate_pose(all_observations)
            object_store.update_id(obj_id, obj)

        except rospy.ServiceException, e:
            print("Service call failed: %s"%(e))
            obj_id = ""
            return (False, obj_id)
    return (True, obj_id)

def estimate_pose(observations):
    ''' Receives a list of observations and returns pose_estimate object

    '''
    pose_estimate = PoseEstimate()
    pose_estimate.most_likely_pose = observations[-1].pose_observation
    pose_estimate.most_recent_pose = observations[-1].pose_observation
    return pose_estimate
