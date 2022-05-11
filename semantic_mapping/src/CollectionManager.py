from http import server
import utils;
from MemoryManager import MemoryManager;

class CollectionManager:
    def __init__(self, ros_type:type, server_loc:str, collection_name:str, memory_manager:MemoryManager):
        self.ros_type:type = ros_type;
        self.server_loc:str = server_loc;
        self.collection_name:str = collection_name;
        self.memory_manager:MemoryManager = memory_manager;

        memory_manager.addCollection(self.collection_name);

    def add_item_to_collection(self, adding):
        adding_dict:dict = utils.obj_to_dict(adding);

        self.memory_manager.collections[self.collection_name].insert_one(adding_dict);        
        pass;

    def get_item_from_collection(self, query):
        pass;