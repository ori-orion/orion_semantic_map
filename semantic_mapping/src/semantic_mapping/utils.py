"""
Author: Matthew Munks
Owner: Matthew Munks
"""

import math
import rospy;
import genpy;
import numpy;
import geometry_msgs.msg

# SESSION_ID = "session_num";
# UID_ENTRY = "entry_uid";
HEADER_ID = "HEADER"
SESSION_ID = "SESSION_NUM";
UID_ENTRY = "UID";
CROSS_REF_UID = "CRSS_REF_UID";

PYMONGO_ID_SPECIFIER = "_id";


# Functions for manipulating the objects directly.
def getHeaderInfoObj(obj_querying):
    try:
        return obj_querying.HEADER;
    except AttributeError:
        rospy.logerr("`" + HEADER_ID + "` is not in the message type given.");
        print();
        raise Exception("`" + HEADER_ID + "` is not in the message type given.");
def getUIDObj(obj_querying):
    header = getHeaderInfoObj(obj_querying);
    try:
        return header.UID;
    except AttributeError:
        rospy.logerr("`UID` is not in `" + HEADER_ID + "`. Please use the SOMHeader.msg type for headers.");
        print();
        raise Exception("`UID` is not in `" + HEADER_ID + "`. Please use the SOMHeader.msg type for headers.");
def getSessionNumObj(obj_querying):
    header = getHeaderInfoObj(obj_querying);
    try:
        return header.SESSION_ID;
    except AttributeError:
        rospy.logerr("`SESSION_NUM` is not in `" + HEADER_ID + "`. Please use the SOMHeader.msg type for headers.");
        print();
        raise Exception("`SESSION_NUM` is not in `" + HEADER_ID + "`. Please use the SOMHeader.msg type for headers.");
def setUIDObj(obj_setting, set_to):
    header = getHeaderInfoObj(obj_setting);
    try:
        header.UID = set_to;
    except AttributeError:
        rospy.logerr("`UID` is not in `" + HEADER_ID + "`. Please use the SOMHeader.msg type for headers.");
        print();
        raise Exception("`UID` is not in `" + HEADER_ID + "`. Please use the SOMHeader.msg type for headers.");
def setSessionNumObj(obj_setting, set_to):
    header = getHeaderInfoObj(obj_setting);
    try:
        header.SESSION_NUM = set_to;
    except AttributeError:
        rospy.logerr("`SESSION_NUM` is not in `" + HEADER_ID + "`. Please use the SOMHeader.msg type for headers.");
        print();
        raise Exception("`SESSION_NUM` is not in `" + HEADER_ID + "`. Please use the SOMHeader.msg type for headers.");
# Functions for manipulating the dictionary objects.
def getHeaderInfoDict(dict_querying:dict) -> dict:
    if HEADER_ID not in dict_querying:
        rospy.logerr("`" + HEADER_ID + "` is not in the message type given.");
        print();
        raise Exception("`" + HEADER_ID + "` is not in the message type given.");
    return dict_querying[HEADER_ID];
def getUIDDict(dict_querying:dict):
    header:dict = getHeaderInfoDict(dict_querying);
    if UID_ENTRY not in header:
        rospy.logerr("`UID` is not in `" + HEADER_ID + "`. Please use the SOMHeader.msg type for headers.");
        print();
        raise Exception("`UID` is not in `" + HEADER_ID + "`. Please use the SOMHeader.msg type for headers.");
    return header[UID_ENTRY];
def getSessionNumDict(dict_querying:dict):
    header:dict = getHeaderInfoDict(dict_querying);
    if SESSION_ID not in header:
        rospy.logerr("`SESSION_NUM` is not in `" + HEADER_ID + "`. Please use the SOMHeader.msg type for headers.");
        print();
        raise Exception("`SESSION_NUM` is not in `" + HEADER_ID + "`. Please use the SOMHeader.msg type for headers.");
    return header[SESSION_ID];
def setUIDDict(dict_setting:dict, set_to):
    header:dict = getHeaderInfoDict(dict_setting);
    if UID_ENTRY not in header:
        rospy.logerr("`UID` is not in `" + HEADER_ID + "`. Please use the SOMHeader.msg type for headers.");
        print();
        raise Exception("`UID` is not in `" + HEADER_ID + "`. Please use the SOMHeader.msg type for headers.");
    header[UID_ENTRY] = set_to;
def setSessionNumDict(dict_setting:dict, set_to):
    header:dict = getHeaderInfoDict(dict_setting);
    if SESSION_ID not in header:
        rospy.logerr("`SESSION_NUM` is not in `" + HEADER_ID + "`. Please use the SOMHeader.msg type for headers.");
        print();
        raise Exception("`SESSION_NUM` is not in `" + HEADER_ID + "`. Please use the SOMHeader.msg type for headers.");
    header[SESSION_ID] = set_to;
# Functions for manipulating the dictionary objects in a non-critical way:
def getHeaderInfoDict_nonCrit(dict_querying:dict) -> dict:
    if HEADER_ID not in dict_querying:
        dict_querying[HEADER_ID] = {};
    return dict_querying[HEADER_ID];


# # Functions for getting and setting time.
# def ROSTimeToNumericalTime(time:rospy.Time) -> int:
#     """
#     Converts rospy.Time (or anything with the fields `secs` and `nsecs`) into a single number.
#     This makes us able to sort w.r.t. the time field.
#     """
#     output = time.nsecs + time.secs * 1e9;
#     return output;
# def numericalTimeToROSTime(time:int) -> rospy.Time:
#     """
#     Converts from the single number representation of time into rospy.Time.
#     """
#     output = rospy.Time();
#     output.nsecs = int(time % 1e9);
#     output.secs = int((time - output.nsecs) / 1e9);
#     # print(output.secs);
#     return output;
# def numericalTimeToROSDuration(time:int) -> rospy.Duration:
#     """
#     Converts from the single number representation of time into rospy.Duration.
#     """
#     output = rospy.Duration();
#     output.nsecs = int(time % 1e9);
#     output.secs = int((time - output.nsecs) / 1e9);
#     # print(output.secs);
#     return output;


# The point here is that we could get either a pose or a point as input. 
# We therefore want to separate these out.
# We also know that the dict obj we might be trying to decipher is a ROS
# Pose object, so we know that "position" is the string we want.
def getPoint(obj:dict) -> numpy.array:
    """
    Returns the numpy.array 3D point from a Point or Pose object.
    """
    if "position" in obj:
        obj = obj["position"];
    return numpy.asarray([obj["x"], obj["y"], obj["z"]]);
def setPoint(obj:dict, new_pt:numpy.array) -> dict:
    """
    Sets the position in either a Point or Pose object from a numpy.array 3D point.
    """
    if ("position" in obj):
        obj["position"]['x'] = new_pt[0];
        obj["position"]['y'] = new_pt[1];
        obj["position"]['z'] = new_pt[2];
    else:
        obj['x'] = new_pt[0];
        obj['y'] = new_pt[1];
        obj['z'] = new_pt[2];
    return obj;

def getMatrix(obj:list, num_rows:int=3) -> numpy.matrix:
    """
    Gets the numpy.matrix from a linear array. 
    Let x=[1,2,3,4,5,6,7,8,9].
    getMatrix(x) -> [[1,2,3],[4,5,6],[7,8,9]]
    """
    obj_array = numpy.asarray(obj);
    obj_2D = obj_array.reshape((num_rows, -1));
    return numpy.matrix(obj_2D);

def quaternion_to_rot_mat(quat:geometry_msgs.msg.Quaternion) -> numpy.array:
    """
    Gets the 3x3 rotation matrix from a quaternion.
    https://automaticaddison.com/how-to-convert-a-quaternion-to-a-rotation-matrix/
    """
    output = numpy.zeros((3,3));

    quat_array = [quat.w, quat.x, quat.y, quat.z];
    quat_0 = quat_array[0];
    for i in range(0,3):
        quat_ip1 = quat_array[i+1];
        output[i,i] = 2 * (quat_0*quat_0 + quat_ip1*quat_ip1) - 1;
    
    # There's probably a nicer way to do this, but for now...
    output[0,1] = 2 * (quat_array[1]*quat_array[2] - quat_array[0]*quat_array[3]);
    output[1,0] = 2 * (quat_array[1]*quat_array[2] + quat_array[0]*quat_array[3]);

    output[0,2] = 2 * (quat_array[1]*quat_array[3] + quat_array[0]*quat_array[2]);
    output[2,0] = 2 * (quat_array[1]*quat_array[3] - quat_array[0]*quat_array[2]);

    output[1,2] = 2 * (quat_array[2]*quat_array[3] - quat_array[0]*quat_array[1]);
    output[2,1] = 2 * (quat_array[2]*quat_array[3] + quat_array[0]*quat_array[1]);
    
    return output;

def get_multi_likelihood(mean:numpy.array, covariance_matrix:numpy.matrix, location:numpy.array) -> numpy.float64:
    """
    Gets the likelihood out from a multidimensional likelihood out for a given position and covariance matrix.
    Note that this is NOT the probability.
    """
    exponent = -0.5 * numpy.dot((mean - location), numpy.matmul(numpy.linalg.inv(covariance_matrix), (mean-location)));
    cov_det = numpy.linalg.det(covariance_matrix);
    return (1/math.sqrt(2*math.pi * cov_det)) * math.exp(exponent);

def get_mean_over_samples(means, covariances) -> numpy.array:
    """
    For MLE, there is an covariance matrix weighted average that is used. This implements that. 

    means           - An array of numpy.array[s]
    covariances     - An array of numpy.matrix[s]
    This will do x = (sum(inv(cov[i])))^(-1) * sum(inv(cov[i])*mean[i]), as per MLE.
    """
    assert(len(means) == len(covariances));
    print("get_mean_over_samples");

    sum_inv_cov = numpy.zeros((3,3));
    # print("\t", sum_inv_cov);
    for i in range(len(means)):
        covariances[i] = numpy.linalg.inv(covariances[i]);
        sum_inv_cov += covariances[i];
        # print("\t", sum_inv_cov);
    
    sum_invcov_mu = numpy.zeros((3,1));
    # print(sum_invcov_mu);
    for i in range(len(means)):
        temp = numpy.asarray(numpy.matmul(covariances[i], means[i])).reshape((3,1));
        #print(temp);
        #print(sum_invcov_mu);
        sum_invcov_mu += temp;

    # inv_sum_inv_cov = numpy.linalg.inv(sum_inv_cov);
    #print(inv_sum_inv_cov);
    #print(sum_invcov_mu);
    output = numpy.matmul(numpy.linalg.inv(sum_inv_cov), sum_invcov_mu);
    return [output[0,0], output[1,0], output[2,0]];


#removes attributes of a ROS msg that we're not interested in.
def get_attributes(obj) -> list:
    """
    Gets a list of the attributes for a given object, and removes those we don't want. 
    ROS message/service types come with a few fields that we don't want, such as the
    header field and the serialize and deserialize functions. These are removed. 
    We also don't want to convert anything with a leading underscore (given that those
    are assigned by default by ROS).
    """
    attributes:list = obj.__dir__();

    def remove_element(e:str):
        if (e in attributes):
            attributes.remove(e);

    remove_element("_get_types");
    remove_element("serialize");
    remove_element("deserialize");
    remove_element("serialize_numpy");
    remove_element("deserialize_numpy");
    remove_element("header");

    i:int = 0;
    while (i < len(attributes)):
        if (attributes[i][0] == '_'):   # If it starts with an underscore!
            attributes.pop(i);  # If we're popping this element, then we don't need to increment the index!
        else:
            i += 1;

    return attributes;

# Main set of infrastructure to convert ROS types to and from dictionaries.
#   should be able to push almost anything into a dictionary (There may well be some as 
#   yet unknown types that need to be dealt with)!
def obj_to_dict(
    obj, 
    attributes:list=None, 
    session_id:int=-1, 
    ignore_default:bool=False, 
    ignore_of_type:list=[],
    convert_caps:bool=False) -> dict:
    
    """
    This will transfer an arbitrary ROS object into a dictionary.

    Inputs:
        obj                     The object to convert. 
        attributes:list         A list of attributes to convert. This is typically used for recursion.
        session_id:int          Sets the session_id of the entry iff session_id != -1.
        ignore_default:bool     For queries, we want to ignore default fields. (If a field is as its 
                                default, then that means we don't want to query into it.)
        ignore_of_type:list     For queries, there are some fields that are a pain (such as temporal 
                                types). This allows us to remove fields from a set of types.
        convert_caps:bool       For queries, we want to convert fields with leading capitals. However,
                                fields with leading capitals should be fields that the system sets 
                                internally. Thus, if we are adding an object, we don't want to set 
                                these fields.
    """
    # print("obj_to_dict(...)");

    if (attributes == None):
        attributes = get_attributes(obj);

    # print("Attributes: {}".format(attributes))

    if (len(attributes) == 0):
        return {};

    # print(attributes, end = ";\n\t");

    def pushObjToDict(element, ignore_default:bool):
        """
        ignore_default is for queries. If we don't want to compare a parameter, we want to 
        be able to set it to the default and ignore it. This sets this up. (The check is
        done right at the end of the function with the comparison output==output_type()).

        Needs testing.
        """

        output = None;
        output_type = None;

        base_types = [str, float, int, bool, complex, bytes, tuple];
        temporal_types = [rospy.Time, rospy.Duration, genpy.rostime.Time];

        if isinstance(element, list):
            adding = [];
            for sub_element in element:
                adding.append(pushObjToDict(sub_element, ignore_default));
            if len(adding) == 0:
                return None;
            return adding;        

        for type_ in base_types:            
            if (type_ not in ignore_of_type) and isinstance(element, type_):
                output = element;
                output_type = type_;
                # rospy.loginfo("Found element '{}', type '{}'".format(element, type_))
        
        if output == None:
            for type_ in temporal_types:
                if (type_ not in ignore_of_type) and isinstance(element, type_):
                    attributes_recursive_in:list = get_attributes(element);
                    output:dict = obj_to_dict(
                        element, 
                        attributes=attributes_recursive_in, 
                        ignore_default=ignore_default, 
                        ignore_of_type=ignore_of_type,
                        convert_caps=convert_caps);
                    output_type = dict;

        if output == None:    
            if (genpy.Message not in ignore_of_type) and isinstance(element, genpy.Message):
                # rospy.loginfo("Found element '{}', type '{}'".format(element, type_))
                attributes_recursive_in:list = get_attributes(element);
                output = obj_to_dict(
                    element, 
                    attributes=attributes_recursive_in, 
                    ignore_default=ignore_default, 
                    ignore_of_type=ignore_of_type,
                    convert_caps=convert_caps);

                output_type = dict;
        
        if ignore_default and output != None and output_type != None:            
            if output == output_type():
                output = None;
        
        return output;
    
    output:dict = {};
    for attr in attributes:
        attr:str;

        # We don't want to look at constants, and constants are all upper case.
        if attr[0].isupper() and not convert_caps: 
            continue;

        element = getattr(obj, attr);
        
        carry = pushObjToDict(element, ignore_default);
        if carry != None:
            output[attr] = carry;

    if (session_id != -1):
        output_header = getHeaderInfoDict_nonCrit(output);
        output_header[SESSION_ID] = session_id;

    return output;

def dict_to_obj(dictionary:dict, objFillingOut):
    """
    The main idea here is that we may well want to convert an arbitrary dictionary to one of the ROS types
    we've created. This will do it.

    Inputs:
        dictionary:dict     The dictionary that we want to fill out the ROS message with. 
        objFillingOut       An empty ROS message to fill out.
    """

    attributes = objFillingOut.__dir__();
    for key in dictionary.keys():
        if (key in attributes):
            if isinstance(dictionary[key], dict):                
                dict_to_obj(dictionary[key], getattr(objFillingOut, key));
                continue;
            elif isinstance(dictionary[key], list):
                carry = [];
                for element in dictionary[key]:
                    if element is dict:
                        raise(Exception("The sub-class of a list is a dictionary. The type is not currently known."));
                    else:
                        carry.append(element);
                setattr(objFillingOut, key, carry);
                continue;
            # elif isinstance(getattr(objFillingOut, key), rospy.Time):
            #     setattr(objFillingOut, key, numericalTimeToROSTime(dictionary[key]));
            # elif isinstance(getattr(objFillingOut, key), rospy.Duration):
            #     setattr(objFillingOut, key, numericalTimeToROSDuration(dictionary[key]));
            # elif isinstance(getattr(objFillingOut, key), genpy.rostime.Time):
            #     setattr(objFillingOut, key, numericalTimeToROSDuration(dictionary[key]));
            else:
                setattr(objFillingOut, key, dictionary[key]);
    
    return objFillingOut;
