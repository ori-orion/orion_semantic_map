#!/usr/bin/env python
import orion_actions.msg
import orion_actions.srv
import rospy
import rosplan_knowledge_msgs.msg 
from MemoryManager import DEBUG, MemoryManager
from CollectionManager import CollectionManager, TypesCollection


def setup_system():
    rospy.init_node('ebb_manager')
    mem_manager = MemoryManager()

    pickplaceresult_types= TypesCollection(
        base_ros_type=orion_actions.msg.PickPlaceResult,
        query_parent=orion_actions.srv.SOMQueryPickPlaceResult,
        query_response=orion_actions.srv.SOMQueryPickPlaceResultResponse,
        input_parent=orion_actions.srv.SOMAddPickPlaceResult,
        input_response=orion_actions.srv.SOMAddPickPlaceResultResponse)

    object_manager = CollectionManager(
        types=pickplaceresult_types,
        service_name="pickplaceresults",
        memory_manager=mem_manager)

    knowledgeitem_types= TypesCollection(
        base_ros_type=orion_actions.msg.KnowledgeItem,
        query_parent=orion_actions.srv.SOMQueryKnowledgeItem,
        query_response=orion_actions.srv.SOMQueryKnowledgeItemResponse,
        input_parent=orion_actions.srv.SOMAddKnowledgeItem,
        input_response=orion_actions.srv.SOMAddKnowledgeItemResponse)

    object_manager = CollectionManager(
        types=knowledgeitem_types,
        service_name="knowledgeitems",
        memory_manager=mem_manager)
    rospy.loginfo("Memory systems set up!")

    rospy.spin()

if __name__ == '__main__':
    setup_system()