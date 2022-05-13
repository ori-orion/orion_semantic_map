import utils;
import pymongo;
import rospy;
from MemoryManager import MemoryManager;

# The root for all things som related.
SERVICE_ROOT = "som/";


class TypesCollection:
    def __init__(self, base_ros_type:type, query_parent:type, query_response:type):
        self.base_ros_type:type = base_ros_type;

        # Services have the parent type, the request type and the response type. 
        # The request can easily just be read in. That makes the response and the
        # parent the only types of relevance here.
        self.query_parent:type = query_parent;
        self.query_response:type = query_response;


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
        

    def add_item_to_collection(self, adding):
        adding_dict:dict = utils.obj_to_dict(adding, ignore_default=False);

        self.collection.insert_one(adding_dict);

    def get_item_from_collection(self, query) -> list:
        query_dict = utils.obj_to_dict(query, ignore_default=True);

        query_result:pymongo.cursor.Cursor = self.collection.find(query_dict);
        query_result_list = list(query_result);

        return query_result_list;

    
    def setup_services(self):

        rospy.Service(
            SERVICE_ROOT + self.service_name + '/input', 
            self.types.base_ros_type, 
            self.add_item_to_collection);

        rospy.Service(
            SERVICE_ROOT + self.service_name + '/basic_query',
            self.types.query_parent,
            self.get_item_from_collection);

        pass;