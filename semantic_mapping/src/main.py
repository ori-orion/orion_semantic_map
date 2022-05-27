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
        memory_manager=mem_manager,
        positional_attr="obj_position"
    );

    observation_types:TypesCollection = TypesCollection(
        base_ros_type=orion_actions.msg.SOMObservation,
        input_parent=orion_actions.srv.SOMAddObservation,
        input_response=orion_actions.srv.SOMAddObservationResponse,
        query_parent=orion_actions.srv.SOMQueryObservations,
        query_response=orion_actions.srv.SOMQueryObservationsResponse
    );
    observation_arg_name_defs:ConsistencyArgs = ConsistencyArgs(
        size_attr="size"
    );
    observation_arg_name_defs.cross_ref_attr.append("class_");
    observation_arg_name_defs.max_distance = 0.3;
    observation_arg_name_defs.first_observed_attr = "first_observed_at";
    observation_arg_name_defs.last_observed_attr = "last_observed_at";
    observation_arg_name_defs.observed_at_attr = "observed_at";
    observation_manager:ConsistencyChecker = ConsistencyChecker(
        pushing_to=object_manager,
        types=observation_types,
        service_name="observations",
        consistency_args=observation_arg_name_defs,
        positional_attr="obj_position"        
    );



    rospy.loginfo("Memory systems set up!");

    rospy.spin();

if __name__ == '__main__':
    setup_system();

    