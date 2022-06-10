#!/usr/bin/env python3

from MemoryManager import MemoryManager;
from CollectionManager import CollectionManager, TypesCollection;
from ObjConsistencyMapper import ConsistencyChecker, ConsistencyArgs;
from RelationManager import RelationManager;
from RegionManager import RegionManager;
import Ontology;

import rospy;

import orion_actions.msg
import orion_actions.srv

import os;

# import semantic_mapping.srv
# import semantic_mapping.msg

# print(__file__);

def setup_system():

    rospy.init_node('som_manager');

    mem_manager:MemoryManager = MemoryManager();
    
    ontology_tree:Ontology.ontology_member = Ontology.read_file(
        os.path.dirname(__file__) + "/labels.txt");
    # This will be a callback within observations for assigning the category of an object.
    def ontology_observation_getCategory_callback(adding_dict:dict, obj_id):
        if (len(adding_dict["category"]) == 0):
            ontological_result = ontology_tree.search_for_term(adding_dict["class_"]);
            if (ontological_result == None):
                ontology_tree.add_term(["unknown",adding_dict["class_"]]);
                adding_dict["category"] = "unknown";
            else:
                print(ontological_result);
                # So this will go [class, category, "Objs"];
                adding_dict["category"] = ontological_result[1];
        return adding_dict, obj_id;
    # ontology_tree.print_graph();

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
        max_distance={"default":0.3, "person":1},
        class_identifier="class_",
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
        consistency_args=observation_arg_name_defs,
        collection_input_callbacks=[ontology_observation_getCategory_callback]
    );

    object_relational_manager:RelationManager = RelationManager(
        operating_on=object_manager,
        positional_attr="obj_position",
        service_base=orion_actions.srv.SOMRelObjQuery,
        service_response=orion_actions.srv.SOMRelObjQueryResponse,
        match_type=orion_actions.msg.Match
    );

    # object_region_types = TypesCollection(
    #     base_ros_type=orion_actions.msg.SOMBoxRegion
    # );
    # object_region_manager:RegionManager = RegionManager(
    #     memory_manager=mem_manager,
    #     types=object_region_types,
    #     service_name="object_regions",
    #     corner_location="corner_loc",
    #     dimension="dimension"
    # );



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



    rospy.loginfo("Memory systems set up!");

    rospy.spin();

if __name__ == '__main__':
    setup_system();

    
