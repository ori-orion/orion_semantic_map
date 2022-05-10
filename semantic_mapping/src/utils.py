import rospy;
import genpy;

SESSION_ID = "session_num";
UID_ENTRY = "entry_uid";



def ROSTimeToNumericalTime(time:rospy.Time) -> int:
    output = time.nsecs + time.secs * 1e9;
    # print(output);
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
def obj_to_dict(obj, add_base_info = True,  attributes:list=None, session_id:int=-1, uid:int=-1) -> dict:
    """
    This will transfer an arbitrary ROS object into a dictionary.
    """
    # print("obj_to_dict(...)");

    if (attributes == None):
        attributes = get_attributes(obj);

    if (len(attributes) == 0):
        return {};

    # print(attributes, end = ";\n\t");

    def pushObjToDict(element):
        # print(type(element));
        if isinstance(element, list):
            adding = [];
            for sub_element in element:
                adding.append(pushObjToDict(sub_element));
            return adding;
        elif (isinstance(element, str) or isinstance(element, float) or isinstance(element, int) or isinstance(element, bool) or isinstance(element, complex) or isinstance(element, bytes)):
            return element;
        elif (isinstance(element, tuple)):
            return element;
        elif isinstance(element, rospy.Time):
            return ROSTimeToNumericalTime(element);            
        elif isinstance(element, rospy.Duration):
            element:rospy.Duration;            
            # I think this works with the current inner working of ROSTimeToNumericalTime(...)
            # Going forward, this might cause problems but not sure.
            return ROSTimeToNumericalTime(element);
        elif isinstance(element, genpy.rostime.Time):
            return ROSTimeToNumericalTime(element);
        elif isinstance(element, genpy.Message):
            # print("Any message");
            attributes_recursive_in:list = get_attributes(element);
            return obj_to_dict(element, False, attributes_recursive_in);
        return None;
    
    output:dict = {};
    for attr in attributes:
        attr:str;

        # We don't want to look at constants, and constants are all upper case.
        if attr[0].isupper(): 
            continue;

        element = getattr(obj, attr);
        
        carry = pushObjToDict(element);
        if carry == None:
            rospy.logwarn(attr + " of type " + str(type(element)) + "could not be added to an entry within the database. ");
        else:
            output[attr] = pushObjToDict(element);        

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
