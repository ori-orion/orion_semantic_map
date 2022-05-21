import rospy;
import genpy;

SESSION_ID = "session_num";
UID_ENTRY = "entry_uid";



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
def obj_to_dict(obj, attributes:list=None, session_id:int=-1, ignore_default:bool=False) -> dict:
    """
    This will transfer an arbitrary ROS object into a dictionary.

    ignore_default is for queries. If we don't want to compare a parameter, we want to 
    be able to set it to the default and ignore it. This sets this up.
    """
    # print("obj_to_dict(...)");

    if (attributes == None):
        attributes = get_attributes(obj);

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


        if isinstance(element, list):
            adding = [];
            for sub_element in element:
                adding.append(pushObjToDict(sub_element, ignore_default));
            return adding;        
        
        for type_ in base_types:            
            if isinstance(element, type_):
                output = element;
                output_type = type_;
        
        if output == None:
            if isinstance(element, rospy.Time):
                output = ROSTimeToNumericalTime(element);
                output_type = rospy.Time;
            elif isinstance(element, rospy.Duration):
                # I think this works with the current inner working of ROSTimeToNumericalTime(...)
                # Going forward, this might cause problems but not sure.
                output = ROSTimeToNumericalTime(element);
                output_type = rospy.Duration;
                                
            elif isinstance(element, genpy.rostime.Time):
                output = ROSTimeToNumericalTime(element);                
                output_type = genpy.rostime.Time;
                
            elif isinstance(element, genpy.Message):                
                attributes_recursive_in:list = get_attributes(element);
                output = obj_to_dict(element, attributes=attributes_recursive_in, ignore_default=ignore_default);
                output_type = dict;
        
        if ignore_default and output != None and output_type != None:            
            if output == output_type():
                output = None;

        return output;
    
    output:dict = {};
    for attr in attributes:
        attr:str;

        # We don't want to look at constants, and constants are all upper case.
        if attr[0].isupper(): 
            continue;

        element = getattr(obj, attr);
        
        carry = pushObjToDict(element, ignore_default);
        if carry == None:
            # rospy.logwarn(attr + " of type " + str(type(element)) + "could not be added to an entry within the database. ");
            pass;
        else:
            # print("\t", attr, "<-", carry);
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

    attributes = objFillingOut.__dir__();
    for key in dictionary.keys():
        if (key in attributes):
            if isinstance(dictionary[key], dict):                
                dict_to_obj(dictionary[key], getattr(objFillingOut, key));
            elif isinstance(dictionary[key], list):
                carry = [];
                for element in dictionary[key]:
                    carry.append(dict_to_obj(element));
                setattr(objFillingOut, key, carry);
                pass;
            elif isinstance(getattr(objFillingOut, key), rospy.Time):   # Needs to be checked
                print("rospy.Time element found.");
                setattr(objFillingOut, key, numericalTimeToROSTime(dictionary[key]));
            elif isinstance(getattr(objFillingOut, key), rospy.Duration):   # Needs to be checked
                print("rospy.Duration element found.");
                setattr(objFillingOut, key, numericalTimeToROSDuration(dictionary[key]));
            else:                
                setattr(objFillingOut, key, dictionary[key]);
    
    # print(objFillingOut);
    return objFillingOut;
