from typing import Any, Optional
import orion_actions.msg as act_msg
from geometry_msgs.msg import Quaternion

import numpy as np
import numpy.typing as npt

from bson.objectid import ObjectId

from new_version.DatabaseTypes import DbObject


def getUidIfPresent(header: act_msg.SOMHeader) -> Optional[ObjectId]:
    """
    Get the UID from a header as a ObjectId, or None if the header is not specified
    """
    uid = header.UID
    if uid == "":
        return None
    return ObjectId(uid)


def getPositionFromDictAsArray(obj_dict: DbObject) -> npt.NDArray:
    """
    Given a DbObject dictionary, this function returns the object position 
    (i.e. `obj_dict["obj_position"]["position"]`) as a Numpy array.

    It raises an exception if the obj_position is not specified in the dict.
    """
    if "obj_position" not in obj_dict:
        raise ValueError(
            "Looking for position in a DbObject that does not contain a pose"
        )
    position = obj_dict["obj_position"]["position"]
    return np.asarray([position["x"], position["y"], position["z"]])

def quaternion_to_rot_mat(quat: Quaternion) -> npt.NDArray:
    """
    Gets the 3x3 rotation matrix from a quaternion.
    https://automaticaddison.com/how-to-convert-a-quaternion-to-a-rotation-matrix/
    """
    output = np.zeros((3,3))

    quat_array = [quat.w, quat.x, quat.y, quat.z]
    quat_0 = quat_array[0]
    for i in range(0,3):
        quat_ip1 = quat_array[i+1]
        output[i,i] = 2 * (quat_0*quat_0 + quat_ip1*quat_ip1) - 1

    # There's probably a nicer way to do this, but for now...
    output[0,1] = 2 * (quat_array[1]*quat_array[2] - quat_array[0]*quat_array[3])
    output[1,0] = 2 * (quat_array[1]*quat_array[2] + quat_array[0]*quat_array[3])

    output[0,2] = 2 * (quat_array[1]*quat_array[3] + quat_array[0]*quat_array[2])
    output[2,0] = 2 * (quat_array[1]*quat_array[3] - quat_array[0]*quat_array[2])

    output[1,2] = 2 * (quat_array[2]*quat_array[3] - quat_array[0]*quat_array[1])
    output[2,1] = 2 * (quat_array[2]*quat_array[3] + quat_array[0]*quat_array[1])
    
    return output