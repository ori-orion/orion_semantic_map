import message_conversion
import rospy

from orion_actions.msg import *
from som_object import InSOMObject


def make_observation(obs, rois, object_store, observation_store):
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
    obs.type = obs.type.lower()

    ## if no object id is supplied insert new object
    if obs.obj_id == "":
        in_som_object = InSOMObject.from_som_observation_message(obs)
        in_som_object.update_from_observation_messages([obs], rois)
        obj = in_som_object.to_som_object_message()

        try:
            obj_id = object_store.insert(obj)
            obj.obj_id = obj_id
            obs.obj_id = obj_id
            obs_id = observation_store.insert(obs)
            object_store.update_id(obj_id, obj)

        except rospy.ServiceException, e:
            print("Service call failed: %s"%(e))
            obj_id = ""
            return (False, obj_id, obj)

    ## if the object id is supplied then update existing
    else:
        try:
            # add observation to store and get all observations of object
            obs_id = observation_store.insert(obs)
            obj_id = obs.obj_id
            response = observation_store.query(SOMObservation._type,message_query={"obj_id": obj_id})
            all_observations = [i[0] for i in response]

            # update object from new observation
            obj, meta = object_store.query_id(obj_id, SOMObject._type)
            in_som_object = InSOMObject.from_som_object_message(obj)
            in_som_object.update_from_observation_messages(all_observations, rois)
            obj = in_som_object.to_som_object_message()
            object_store.update_id(obj_id, obj)

        except rospy.ServiceException, e:
            print("Service call failed: %s"%(e))
            obj_id = ""
            return (False, obj_id, obj)
    return (True, obj_id, obj)
