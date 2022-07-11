import math;
import rospy;
import genpy;
import numpy;
import geometry_msgs.msg

# SESSION_ID = "session_num";
# UID_ENTRY = "entry_uid";
SESSION_ID = "SESSION_NUM";
UID_ENTRY = "UID";
CROSS_REF_UID = "CRSS_REF_UID";

PYMONGO_ID_SPECIFIER = "_id";


def ROSTimeToNumericalTime(time:rospy.Time) -> int:
    output = time.nsecs + time.secs * 1e9;
    return output;
def numericalTimeToROSTime(time:int) -> rospy.Time:
    output = rospy.Time();
    output.nsecs = int(time % 1e9);
    output.secs = int((time - output.nsecs) / 1e9);
    # print(output.secs);
    return output;
def numericalTimeToROSDuration(time:int) -> rospy.Duration:
    output = rospy.Duration();
    output.nsecs = int(time % 1e9);
    output.secs = int((time - output.nsecs) / 1e9);
    # print(output.secs);
    return output;


# The point here is that we could get either a pose or a point as input. 
# We therefore want to separate these out.
# We also know that the dict obj we might be trying to decipher is a ROS
# Pose object, so we know that "position" is the string we want.
def getPoint(obj:dict) -> numpy.array:
    if "position" in obj:
        obj = obj["position"];
    return numpy.asarray([obj["x"], obj["y"], obj["z"]]);
def setPoint(obj:dict, new_pt:numpy.array) -> dict:
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
    obj_array = numpy.asarray(obj);
    obj_2D = obj_array.reshape((num_rows, -1));
    return numpy.matrix(obj_2D);

def quaternion_to_rot_mat(quat:geometry_msgs.msg.Quaternion) -> numpy.array:
    """
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
    exponent = -0.5 * numpy.dot((mean - location), numpy.matmul(numpy.linalg.inv(covariance_matrix), (mean-location)));
    cov_det = numpy.linalg.det(covariance_matrix);
    return (1/math.sqrt(2*math.pi * cov_det)) * math.exp(exponent);

def get_mean_over_samples(means, covariances) -> numpy.array:
    """
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
    ignore_of_type=[],
    convert_caps=False) -> dict:
    
    """
    This will transfer an arbitrary ROS object into a dictionary.

    ignore_default is for queries. If we don't want to compare a parameter, we want to 
    be able to set it to the default and ignore it. This sets this up.

    For queries, there are fields of types we want to ignore completely (such as those of time.)
        The ignore_of_type entry handles this.
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
                    output = ROSTimeToNumericalTime(element);
                    output_type = type_;

        if output == None:    
            if (genpy.Message not in ignore_of_type) and isinstance(element, genpy.Message):                
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
        
        # rospy.loginfo("Ended with output: '{}'".format(output))
        return output;
    
    output:dict = {};
    for attr in attributes:
        attr:str;

        # We don't want to look at constants, and constants are all upper case.
        if attr[0].isupper() and not convert_caps: 
            continue;

        element = getattr(obj, attr);
        # rospy.loginfo("Element started as '{}', element: '{}'".format(element,attr));

        
        carry = pushObjToDict(element, ignore_default);
        if carry == None:
            # rospy.logwarn(attr + " of type " + str(type(element)) + "could not be added to an entry within the database. ");
            pass;
        else:
            # rospy.loginfo("\t" + attr + "<-" + str(carry));
            output[attr] = carry;

    if (session_id != -1):
        output[SESSION_ID] = session_id;

    return output;

def dict_to_obj(dictionary:dict, objFillingOut):
    """
    The main idea here is that we may well want to convert an arbitrary dictionary to one of the ROS types
    we've created. This will do it.
    """
    # print("obj_to_dict(...)");
    # print(dictionary);
    # print(type(objFillingOut));

    temporal_types = [rospy.Time, rospy.Duration, genpy.rostime.Time];

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
                # print("Setting", key, "=", carry);
                setattr(objFillingOut, key, carry);
                continue;
            elif isinstance(getattr(objFillingOut, key), rospy.Time):   # Needs to be checked
                # print("rospy.Time element found.");
                # print("Setting",key, "[time]");
                setattr(objFillingOut, key, numericalTimeToROSTime(dictionary[key]));
            elif isinstance(getattr(objFillingOut, key), rospy.Duration):   # Needs to be checked
                # print("rospy.Duration element found.");
                # print("Setting",key, "[time]");
                setattr(objFillingOut, key, numericalTimeToROSDuration(dictionary[key]));
            elif isinstance(getattr(objFillingOut, key), genpy.rostime.Time):   # Needs to be checked
                # print("rospy.Duration element found.");
                # print("Setting",key, "[time]");
                setattr(objFillingOut, key, numericalTimeToROSDuration(dictionary[key]));
            else:
                # print("Setting",key,"=", dictionary[key]);
                setattr(objFillingOut, key, dictionary[key]);
    
    # print(objFillingOut);
    return objFillingOut;


if __name__ == '__main__':
    output = quaternion_to_rot_mat(geometry_msgs.msg.Quaternion(x=1,y=1));
    print(output);
