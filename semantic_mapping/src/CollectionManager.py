from turtle import position
import utils;
import pymongo;
import pymongo.collection
import rospy;
import genpy;
from MemoryManager import MemoryManager, DEBUG, SESSION_ID, UID_ENTRY;

# The root for all things som related.
SERVICE_ROOT = "som/";

class TypesCollection:
    def __init__(self, 
        base_ros_type:type, 
        input_parent:type=None,
        input_response:type=None,
        query_parent:type=None, 
        query_response:type=None):

        self.base_ros_type:type = base_ros_type;

        # Services have the parent type, the request type and the response type. 
        # The request can easily just be read in. That makes the response and the
        # parent the only types of relevance here.
        self.query_parent:type = query_parent;
        self.query_response:type = query_response;

        # The service definition for the input type.
        self.input_parent:type = input_parent;
        self.input_response:type = input_response;


class CollectionManager:
    """
    This will be the collection level interface into pymongo, as well as dealing with the basic
    service queries in and out of the collection.

    ros_type        - The base ROS type that this inherits from. Can be None (in the case where
                        we are taking observations and so need to do stuff with it beforehand).
    service_name    - The rossrv list name of the service. This is the service name that you
                        will be able to send stuff to in order to add it to this collection.
                        Note that this will also be the name of the pymongo collection.    
    memory_manager  - The interface through which the basic memory mangement will happen.                            
    """
    def __init__(self, types:TypesCollection, service_name:str, memory_manager:MemoryManager):
        self.types:TypesCollection = types;
        self.service_name:str = service_name;        
        self.memory_manager:MemoryManager = memory_manager;

        # Makes sure the collection is added to the memory manager.
        self.collection:pymongo.collection.Collection = memory_manager.addCollection(self.service_name);        

        self.collection_input_callbacks = [];

        self.setupServices();

    def addItemToCollectionDict(self, adding_dict:dict) -> pymongo.collection.ObjectId:
        adding_dict[SESSION_ID] = self.memory_manager.current_session_id;

        if (DEBUG):
            rospy.logdebug("adding object...");
            print(adding_dict);

        # This is for inserting stuff into the higher level system.
        for callback in self.collection_input_callbacks:
            callback(adding_dict);

        result = self.collection.insert_one(adding_dict);

        result_id:pymongo.collection.ObjectId = result.inserted_id;
        # print(result_id);
        return result_id;

    def addItemToCollection(self, adding) -> pymongo.collection.ObjectId:
        adding_dict:dict = utils.obj_to_dict(adding, ignore_default=False);
        return self.addItemToCollectionDict(adding_dict);

    def rosPushToCollection(self, pushing): # -> self.types.input_response
        pushing_attr = utils.get_attributes(pushing);
        rospy.logdebug("obj adding has name " + pushing_attr[0] + ".");
        uid = self.addItemToCollection(getattr(pushing, pushing_attr[0]));

        response = self.types.input_response();
        # this needs to have the field UID in it!
        response.UID = str(uid);
        return response;
        
    def updateEntry(self, uid:pymongo.collection.ObjectId, update_to:dict):
        """
        https://www.w3schools.com/python/python_mongodb_update.asp

        So we can use
        set = { "$set": { "address": "Canyon 123" } }
        to set the address of the entry to.

        My guess is that you can update the entire entry by simply having
        set = { "address": "Canyon 123" }
        but this is a guess at present.

        Note also, _id is an internal mongodb convention
        """

        self.collection.update_one({utils.PYMONGO_ID_SPECIFIER:uid}, update_to);
        pass


    def queryIntoCollection(self, query_dict) -> list:
        # query_dict = utils.obj_to_dict(query, ignore_default=True);

        if (DEBUG):
            rospy.logdebug("querying object...");
            print(query_dict);

        query_result:pymongo.cursor.Cursor = self.collection.find(query_dict);
        query_result_list = list(query_result);

        return query_result_list;

    def rosQueryEntrypoint(self, ros_query):    # -> self.types.query_response
        ros_query_dict:dict = utils.obj_to_dict(
            ros_query, 
            ignore_default=True,
            ignore_of_type=[rospy.Time, rospy.Duration, genpy.rostime.Time]
        );

        response:list = self.queryIntoCollection(ros_query_dict[list(ros_query_dict.keys())[0]]);

        ros_response = self.types.query_response();
        query_response_attr = utils.get_attributes(ros_response)[0];

        resp_array = getattr(ros_response, query_response_attr);

        assert(type(resp_array) is list);

        for element in response:
            if DEBUG:
                print(element);

            appending = self.types.base_ros_type();
            appending = utils.dict_to_obj(element, appending);
            resp_array.append(appending);

        return ros_response;        

    
    def setupServices(self):

        if (self.types.input_parent != None):
            rospy.Service(
                SERVICE_ROOT + self.service_name + '/input', 
                self.types.input_parent, 
                self.rosPushToCollection);
        
        if (self.types.query_parent != None and self.types.query_response != None):
            rospy.Service(
                SERVICE_ROOT + self.service_name + '/basic_query',
                self.types.query_parent,
                self.rosQueryEntrypoint);

        pass;