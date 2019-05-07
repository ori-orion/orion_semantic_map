
from orion_actions.msg import SOMObservation, SOMObject


def observation_to_object(obs, default_frame_id):
    obj = SOMObject()

    if (obj.header.frame_id == ""):
        obj.header.frame_id = "/map"

    if(obj.cloud.header.frame_id == ""):
        obj.cloud.header.frame_id = "/map"

    obj.map_name = obs.map_name
    obj.meta_properties = obs.meta_properties
    obj.type = obs.type
    obj.size = obs.size
    obj.weight = obs.weight
    obj.task_role = obs.task_role
    obj.robot_pose = obs.robot_pose
    obj.cloud = obs.cloud
    obj.colour = obs.colour
    obj.room_name = obs.room_name
    obj.waypoint = obs.waypoint
    obj.room_geometry = obs.room_geometry
    obj.name = obs.name
    obj.age = obs.age
    obj.posture = obs.posture
    obj.gender = obs.gender
    obj.shirt_colour = obs.shirt_colour

    return obj
