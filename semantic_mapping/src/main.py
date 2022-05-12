from symtable import Function
from MemoryManager import MemoryManager;
from CollectionManager import CollectionManager;

import rospy;

def setup_system():

    rospy.init_node('som_manager');

    mem_manager:MemoryManager = MemoryManager();

    # observation_manager:CollectionManager = CollectionManager();