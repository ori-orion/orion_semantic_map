#!/usr/bin/env python3

from MemoryManager import MemoryManager;
from CollectionManager import CollectionManager, TypesCollection;
from ObjConsistencyMapper import ConsistencyChecker, ConsistencyArgs;
from RelationManager import RelationManager;
import utils;

import rospy;

import orion_actions.msg
import orion_actions.srv
import geometry_msgs.msg;

# import semantic_mapping.srv
# import semantic_mapping.msg

def setup_system():

    rospy.init_node('som_manager');

    mem_manager:MemoryManager = MemoryManager();

    
    # The human stuff needs to be before the object stuff if we are to push
    # humans as a result of seeing objects.
    human_types:TypesCollection = TypesCollection(
        base_ros_type=orion_actions.msg.Human,
        query_parent=orion_actions.srv.SOMQueryHumans,
        query_response=orion_actions.srv.SOMQueryHumansResponse
    );
    human_manager:CollectionManager = CollectionManager(
        types=human_types,
        service_name="humans",
        memory_manager=mem_manager
    );

    human_observation_types:TypesCollection = TypesCollection(
        base_ros_type=orion_actions.msg.HumanObservation,
        input_parent=orion_actions.srv.SOMAddHumanObs,
        input_response=orion_actions.srv.SOMAddHumanObsResponse
    );
    human_observation_manager_args:ConsistencyArgs = ConsistencyArgs(
        position_attr="obj_position",
        first_observed_attr="first_observed_at",
        last_observed_attr="last_observed_at",
        observed_at_attr="observed_at"
    );
    human_observation_manager_args.use_running_average_position = False;
    # human_observation_manager_args.cross_ref_attr.append("task_role");
    human_observation_manager_args.cross_ref_attr.append("object_uid");
    human_observation_manager:ConsistencyChecker = ConsistencyChecker(
        pushing_to=human_manager,
        types=human_observation_types,
        service_name="human_observations",
        consistency_args=human_observation_manager_args
    );
    

    object_types:TypesCollection = TypesCollection(
        base_ros_type=orion_actions.msg.SOMObject,
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
        size_attr="size",
        max_distance=0.3,
        first_observed_attr="first_observed_at",
        last_observed_attr="last_observed_at",
        observed_at_attr="observed_at",
        observation_batch_num="observation_batch_num",
        last_observation_batch="last_observation_batch"
    );
    observation_arg_name_defs.cross_ref_attr.append("class_");
    observation_manager:ConsistencyChecker = ConsistencyChecker(
        pushing_to=object_manager,
        types=observation_types,
        service_name="observations",
        consistency_args=observation_arg_name_defs           
    );
    
    def push_person_callback(adding:dict, obj_uid:str):
        if adding["class_"] == "person":
            human_query:list = human_manager.queryIntoCollection({"object_uid":obj_uid});
            # So we want there to be one entry that's consistent with this object_uid.
            # Note that "object_uid" is what is being checked for consistency so
            # there should never be more than 1. 
            if len(human_query) == 0:
                adding_human = orion_actions.msg.HumanObservation();
                adding_human.object_uid = obj_uid;
                adding_human.obj_position = utils.dict_to_obj(adding["obj_position"], geometry_msgs.msg.Pose());
                adding_human.observed_at = utils.numericalTimeToROSTime(adding["observed_at"]);
                human_observation_manager.addItemToCollectionDict(
                    utils.obj_to_dict(adding_human));

        return adding, obj_uid;

    # This needs to be here because we need the callback to be called AFTER `obj_uid`
    # has been assigned by the observation_manager. Thus the Observation manager 
    # needs to be created first.
    observation_manager.collection_input_callbacks.append(push_person_callback);

    object_relational_manager:RelationManager = RelationManager(
        operating_on=object_manager,
        positional_attr="obj_position",
        service_base=orion_actions.srv.SOMRelObjQuery,
        service_response=orion_actions.srv.SOMRelObjQueryResponse,
        match_type=orion_actions.msg.Match
    );

    rospy.loginfo("Memory systems set up!");

    rospy.spin();

if __name__ == '__main__':
    setup_system();

    
