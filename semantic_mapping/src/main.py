from MemoryManager import MemoryManager;
from CollectionManager import CollectionManager, TypesCollection;

import rospy;

import orion_actions.msg

def setup_system():

    rospy.init_node('som_manager');

    mem_manager:MemoryManager = MemoryManager();

    observation_types:TypesCollection = TypesCollection(orion_actions.msg.SOMObservation)
    observation_manager:CollectionManager = CollectionManager()