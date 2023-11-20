from bson import ObjectId
import rospy
import genpy
from geometry_msgs.msg import Pose, Quaternion, Point

import orion_actions.msg as act_msg

from typing import Optional, TypedDict, List

from new_version.constants import HUMAN_CLASS, PYMONGO_ID_KEY


class DbOrientation(TypedDict):
    w: float
    x: float
    y: float
    z: float


class DbVec3D(TypedDict):
    x: float
    y: float
    z: float


class DbPose(TypedDict):
    orientation: DbOrientation
    position: DbVec3D


class DbHeader(TypedDict):
    SESSION_NUM: int


class DbTime(TypedDict):
    secs: int
    nsecs: int


class DbHumanProperties(TypedDict, total=False):
    face_id: str
    gender: str
    height: float
    name: str
    pronouns: str
    spoken_to_state: int
    task_role: str
    face_attributes: List[str]


class DbObject(TypedDict, total=False):
    # This should be the same as objects
    # TODO: observed_at not added, since it will always be the same as last_observed_at. Do we need it?
    # TODO: Do we need last_observation batch? (Kept for now)
    HEADER: DbHeader
    _id: ObjectId
    base_pose: DbPose
    camera_pose: DbPose
    category: str
    class_: str
    colour: str
    covariance_mat: List[float]
    is_static: bool
    obj_position: DbPose
    picked_up: bool
    pickupable: bool
    size: DbVec3D
    transform_cov_to_diagonal: List[float]
    first_observed_at: DbTime
    last_observed_at: DbTime
    last_observation_batch: int
    num_observations: int
    tf_name: str
    human_properties: DbHumanProperties


def convertQuaternionToDict(quat: Quaternion, even_if_empty = True) -> Optional[DbOrientation]:
    if not even_if_empty and quat.x == quat.y == quat.z == quat.w == 0:
        return None
    return {"x": quat.x, "y": quat.y, "z": quat.z, "w": quat.w}


def convertDictToQuaternion(quat_dict: DbOrientation) -> Quaternion:
    return Quaternion(
        x=quat_dict["x"], y=quat_dict["y"], z=quat_dict["z"], w=quat_dict["w"]
    )


def convertPointToDict(point: Point, even_if_empty= True) -> Optional[DbVec3D]:
    if not even_if_empty and point.x == point.y == point.z == 0:
        return None
    return {"x": point.x, "y": point.y, "z": point.z}


def convertDictToPoint(point_dict: DbVec3D) -> Point:
    return Point(x=point_dict["x"], y=point_dict["y"], z=point_dict["z"])


def convertPoseToDict(pose: Pose, even_if_empty = True) -> Optional[DbPose]:
    orientation = convertQuaternionToDict(pose.orientation, even_if_empty)
    position = convertPointToDict(pose.position, even_if_empty)
    if not even_if_empty and orientation is None or position is None:
        return None
    return {
        "orientation": orientation,
        "position": position,
    } # type: ignore


def convertDictToPose(pose_dict: DbPose) -> Pose:
    position = convertDictToPoint(pose_dict["position"])
    orientation = convertDictToQuaternion(pose_dict["orientation"])
    return Pose(orientation=orientation, position=position)


def convertTimeToDict(tm: genpy.rostime.Time, even_if_empty = True) -> Optional[DbTime]:
    if not even_if_empty and tm.secs == tm.nsecs == 0:
        return None
    return {"secs": tm.secs, "nsecs": tm.nsecs}

def convertDictToTime(tm_dict: DbTime) -> genpy.rostime.Time:
    return genpy.rostime.Time(secs=tm_dict["secs"], nsecs=tm_dict["nsecs"])


def convertObservationToObjectDict(
    observation: act_msg.SOMObservation,
    category: Optional[str] = None,
    pickupable: Optional[bool] = None,
    tf_name: Optional[str] = None,
    is_human=False,
) -> DbObject:
    # TODO: deal with HEADER, human properties. For now human properties are basically hardcoded. 
    #       The HEADER is not set here, so it should be set by the caller.
    observed_at = convertTimeToDict(observation.observed_at)
    category = observation.category if category is None else category
    obj_dict: DbObject = {
        "base_pose": convertPoseToDict(observation.base_pose),
        "camera_pose": convertPoseToDict(observation.camera_pose),
        "category": category,
        "class_": observation.class_,
        "colour": observation.colour,
        "covariance_mat": list(observation.covariance_mat),
        "first_observed_at": observed_at,
        "is_static": observation.is_static,
        "last_observation_batch": observation.observation_batch_num,
        "last_observed_at": observed_at,
        "num_observations": 1,
        "obj_position": convertPoseToDict(observation.obj_position),
        "picked_up": observation.picked_up,
        "size": convertPointToDict(observation.size),
        "transform_cov_to_diagonal": list(observation.transform_cov_to_diagonal),
    } # type: ignore

    if pickupable is not None:
        obj_dict["pickupable"] = pickupable
    if tf_name is not None:
        obj_dict["tf_name"] = tf_name
    if is_human:
        # TODO: For consistency with old SOM we always add everything. We may want to
        #       only add the properties that are not empty (possibly more efficient?)
        #       On the other hand, this allows for more consistency between objects in the DB
        obj_dict["human_properties"] = {
            "face_id": "",
            "gender": "",
            "height": obj_dict["size"]["z"], # type: ignore
            "name": "",
            "pronouns": "",
            "spoken_to_state": act_msg.Human._NOT_SPOKEN_TO,
            "task_role": "",
            "face_attributes": []
        }

    return obj_dict

def convertObjectToDbQuery(obj: act_msg.SOMObject) -> DbObject:
    query: DbObject = {}
    if obj.HEADER.SESSION_NUM != 0:
        query["HEADER"] = {"SESSION_NUM": obj.HEADER.SESSION_NUM}
    if obj.HEADER.UID != "":
        query["_id"] = ObjectId(obj.HEADER.UID)
    if obj.class_ != "":
        query["class_"] = obj.class_
    if obj.colour != "":
        query["colour"] = obj.colour
    if obj.tf_name != "":
        query["tf_name"] = obj.tf_name
    if obj.last_observation_batch != 0:
        query["last_observation_batch"] = obj.last_observation_batch
    if obj.num_observations != 0:
        query["num_observations"] = obj.num_observations
    if obj.category != "":
        query["category"] = obj.category         
    if obj.pickupable == False :
        # TODO: this doesn't work, it will always remove it even if it is set to False
        query["pickupable"] = obj.pickupable
    if obj.picked_up == False:
        query["picked_up"] = obj.picked_up
    if obj.is_static == False:
        query["is_static"] = obj.is_static
    
    obj_position = convertPoseToDict(obj.obj_position, even_if_empty=False)
    if obj_position is not None:
        query["obj_position"] = obj_position

    first_observed_at = convertTimeToDict(obj.first_observed_at, even_if_empty=False)
    if first_observed_at is not None:
        query["first_observed_at"] = first_observed_at

    last_observed_at = convertTimeToDict(obj.last_observed_at, even_if_empty=False)
    if last_observed_at is not None:
        query["last_observed_at"] = last_observed_at

    size = convertPointToDict(obj.size, even_if_empty=False)
    if size is not None:
        query["size"] = size    
    
    return query

def convertHumanToDbQuery(obj: act_msg.Human) -> DbObject:
    query: DbObject = {}
    if obj.HEADER.SESSION_NUM != 0:
        query["HEADER"] = {"SESSION_NUM": obj.HEADER.SESSION_NUM}
    if obj.object_uid != "":
        query["_id"] = ObjectId(obj.object_uid)
    
    query["class_"] = HUMAN_CLASS
   
    if obj.last_observation_batch != 0:
        query["last_observation_batch"] = obj.last_observation_batch

    
    obj_position = convertPoseToDict(obj.obj_position, even_if_empty=False)
    if obj_position is not None:
        query["obj_position"] = obj_position

    first_observed_at = convertTimeToDict(obj.first_observed_at, even_if_empty=False)
    if first_observed_at is not None:
        query["first_observed_at"] = first_observed_at

    last_observed_at = convertTimeToDict(obj.last_observed_at, even_if_empty=False)
    if last_observed_at is not None:
        query["last_observed_at"] = last_observed_at

    human_properties: DbHumanProperties = {}
    if obj.task_role != "":
        human_properties["task_role"] = obj.task_role
    if obj.height != 0:
        human_properties["height"] = obj.height
    if obj.name != "":
        human_properties["name"] = obj.name
    if obj.gender != "":
        human_properties["gender"] = obj.gender
    if obj.pronouns != "":
        human_properties["pronouns"] = obj.pronouns
    if obj.face_id != "":
        human_properties["face_id"] = obj.face_id
    if obj.spoken_to_state != 0:
        human_properties["spoken_to_state"] = obj.spoken_to_state
    query["human_properties"] = human_properties
    
    return query

def convertDictToObject(obj_dict: DbObject) -> act_msg.SOMObject:
    header = act_msg.SOMHeader(SESSION_NUM=obj_dict["HEADER"]["SESSION_NUM"], #type: ignore
                                   UID=obj_dict[PYMONGO_ID_KEY]) #type: ignore
    return act_msg.SOMObject(
        HEADER = header, #type: ignore
        class_ = obj_dict["class_"],#type: ignore
        colour = obj_dict["colour"],#type: ignore
        tf_name = obj_dict["tf_name"], #type: ignore
        num_observations = obj_dict["num_observations"], #type: ignore
        last_observation_batch = obj_dict["last_observation_batch"], #type: ignore
        category = obj_dict["category"], #type: ignore
        obj_position = convertDictToPose(obj_dict["obj_position"]), #type: ignore
        pickupable = obj_dict["pickupable"], #type: ignore
        picked_up = obj_dict["picked_up"], #type: ignore
        first_observed_at = convertDictToTime(obj_dict["first_observed_at"]),  #type: ignore
        last_observed_at = convertDictToTime(obj_dict["last_observed_at"]),  #type: ignore
        size = convertDictToPoint(obj_dict["size"]), #type: ignore
        is_static = obj_dict["is_static"], #type: ignore
        covariance_mat = obj_dict["covariance_mat"] # type: ignore
    )

def convertDictToHuman(obj_dict: DbObject) -> act_msg.Human:
    header = act_msg.SOMHeader(SESSION_NUM=obj_dict["HEADER"]["SESSION_NUM"], #type: ignore
                                   UID=obj_dict[PYMONGO_ID_KEY]) #type: ignore
    human_properties: DbHumanProperties = obj_dict["human_properties"] #type: ignore
    return act_msg.Human(
        header = header,
        task_role = human_properties["task_role"], #type: ignore
        height = human_properties["height"], #type: ignore
        obj_position = convertDictToPose(obj_dict["obj_position"]), #type: ignore
        first_observed_at = convertDictToTime(obj_dict["first_observed_at"]), # type: ignore
        last_observed_at = convertDictToTime(obj_dict["last_observed_at"]), #type: ignore
        object_uid = str(obj_dict[PYMONGO_ID_KEY]), #type: ignore
        name = human_properties["name"], # type: ignore
        gender = human_properties["gender"], #type: ignore
        pronouns = human_properties["pronouns"], #type: ignore
        face_id = human_properties["face_id"], #type: ignore 
        face_attributes = human_properties["face_attributes"], #type: ignore
        spoken_to_state = human_properties["spoken_to_state"] # type: ignore
    )