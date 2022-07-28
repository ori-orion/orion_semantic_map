#!/usr/bin/env python
import orion_actions.msg
import orion_actions.srv
import rospy

from MemoryManager import DEBUG, MemoryManager
from CollectionManager import CollectionManager, TypesCollection


def setup_system():
    rospy.init_node('ebb_manager')
    mem_manager = MemoryManager()

    location_types= TypesCollection(
        base_ros_type=orion_actions.msg.Location,
        query_parent=orion_actions.srv.QueryLocation,
        query_response=orion_actions.srv.QueryLocationResponse,
        input_parent=orion_actions.srv.AddLocation,
        input_response=orion_actions.srv.AddLocationResponse)

    object_manager = CollectionManager(
        types=location_types,
        service_name="locations",
        memory_manager=mem_manager)
    rospy.loginfo("Memory systems set up!")

    rospy.spin()

if __name__ == '__main__':
    setup_system()