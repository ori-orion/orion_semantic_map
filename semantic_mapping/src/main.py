#!/usr/bin/env python3

from MemoryManager import MemoryManager;
from CollectionManager import CollectionManager, TypesCollection;
from ObjConsistencyMapper import ConsistencyChecker, ConsistencyArgs;

import rospy;

import orion_actions.msg
import orion_actions.srv

# import semantic_mapping.srv
# import semantic_mapping.msg

def setup_system():

    rospy.init_node('som_manager');

    mem_manager:MemoryManager = MemoryManager();
    
    object_types:TypesCollection = TypesCollection(
        base_ros_type=orion_actions.msg.SOMObject_new,
        query_parent=orion_actions.srv.SOMQueryObjects,
        query_response=orion_actions.srv.SOMQueryObjectsResponse
    );
    object_manager:CollectionManager = CollectionManager(
        object_types,
        "objects",
        memory_manager=mem_manager
    );

    observation_types:TypesCollection = TypesCollection(
        base_ros_type=orion_actions.msg.SOMObservation,
        input_parent=orion_actions.srv.SOMAddObservation,
        input_response=orion_actions.srv.SOMAddObservationResponse,
        query_parent=orion_actions.srv.SOMQueryObservations,
        query_response=orion_actions.srv.SOMQueryObservationsResponse
    );
    observation_arg_name_defs:ConsistencyArgs = ConsistencyArgs(
        position_attr="obj_position",
        size_attr="size"
    );
    observation_arg_name_defs.cross_ref_attr.append("class_");
    observation_arg_name_defs.max_distance = 0.3;
    observation_manager:ConsistencyChecker = ConsistencyChecker(
        pushing_to=object_manager,
        types=observation_types,
        service_name="observations",
        consistency_args=observation_arg_name_defs
    );



    rospy.loginfo("Memory systems set up!");

    rospy.spin();

if __name__ == '__main__':
    setup_system();

    