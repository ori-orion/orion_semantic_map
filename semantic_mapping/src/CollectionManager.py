import utils;
import pymongo;
import rospy;
from MemoryManager import MemoryManager;

# The root for all things som related.
SERVICE_ROOT = "som/";


class TypesCollection:
    def __init__(self, 
        base_ros_type:type, 
        input_parent:type=None, 
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
        self.collection = memory_manager.addCollection(self.service_name);
        

    def addItemToCollection(self, adding):
        adding_dict:dict = utils.obj_to_dict(adding, ignore_default=False);

        self.collection.insert_one(adding_dict);

    def queryIntoCollection(self, query_dict) -> list:
        # query_dict = utils.obj_to_dict(query, ignore_default=True);

        query_result:pymongo.cursor.Cursor = self.collection.find(query_dict);
        query_result_list = list(query_result);

        return query_result_list;

    def rosQueryEntrypoint(self, ros_query):    # -> self.types.query_response
        ros_query_dict:dict = utils.obj_to_dict(ros_query, ignore_default=True);

        response:list = self.queryIntoCollection(ros_query_dict[ros_query_dict.keys()[0]]);

        ros_response = self.types.query_response();
        query_response_attr = utils.get_attributes(ros_response)[0];

        resp_array = getattr(ros_response, query_response_attr);

        assert(type(resp_array) is list);

        for element in response:
            appending = self.types.base_ros_type();
            appending = utils.dict_to_obj(element, appending);
            resp_array.append(appending);

        return ros_response;        

    
    def setupServices(self):

        if (self.types.input_parent != None):
            rospy.Service(
                SERVICE_ROOT + self.service_name + '/input', 
                self.types.input_parent, 
                self.addItemToCollection);
        
        if (self.types.query_parent != None and self.types.query_response != None):
            rospy.Service(
                SERVICE_ROOT + self.service_name + '/basic_query',
                self.types.query_parent,
                self.rosQueryEntrypoint);

        pass;