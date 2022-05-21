#!/usr/bin/env python3

from MemoryManager import MemoryManager;
from CollectionManager import CollectionManager, TypesCollection;

import rospy;

import orion_actions.msg
import orion_actions.srv

# import semantic_mapping.srv
# import semantic_mapping.msg


def setup_system():

    rospy.init_node('som_manager');

    mem_manager:MemoryManager = MemoryManager();

    observation_types:TypesCollection = TypesCollection(
        base_ros_type=orion_actions.msg.SOMObservation,
        input_parent=orion_actions.srv.SOMAddObservation,
        input_response=orion_actions.srv.SOMAddObservationResponse,
        query_parent=orion_actions.srv.SOMGetObservations,
        query_response=orion_actions.srv.SOMGetObservationsResponse
    );
    observation_manager:CollectionManager = CollectionManager(
        observation_types,
        "observations",
        memory_manager=mem_manager
    );

    rospy.loginfo("Memory systems set up!");

    rospy.spin();

if __name__ == '__main__':
    setup_system();

    