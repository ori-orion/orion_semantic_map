#!/usr/bin/env python3
"""
Author: Matthew Munks
Owner: Matthew Munks
"""

from interactive_markers.interactive_marker_server import InteractiveMarkerServer

from MemoryManager import DEBUG, MemoryManager;
from CollectionManager import CollectionManager, TypesCollection;
from ObjConsistencyMapper import ConsistencyChecker, ConsistencyArgs;
from RelationManager import RelationManager;
from RegionManager import RegionManager;
import Ontology;
from visualisation import RvizVisualisationManager;
import utils;

import rospy;

import orion_actions.msg
import orion_actions.srv
import geometry_msgs.msg;

import os;

# import semantic_mapping.srv
# import semantic_mapping.msg

# print(__file__);


NUM_OBSERVATIONS_THRESHOLD_FOR_QUERY = 0;


def setup_system():

    rospy.init_node('som_manager');

    mem_manager:MemoryManager = MemoryManager();

    interactive_marker_server = InteractiveMarkerServer("zzz_som/obj_vis")
    interactive_marker_server_regions = InteractiveMarkerServer("zzz_som/region_vis")
    interactive_marker_server_humans = InteractiveMarkerServer("zzz_som/human_vis")
    
    ontology_tree:Ontology.ontology_member = Ontology.read_file(
        os.path.dirname(__file__) + "/labels.txt");
    # This will be a callback within observations for assigning the category of an object.
    def ontology_observation_getCategory_callback(adding_dict:dict, metadata:dict):
        if (len(adding_dict["category"]) == 0):
            ontological_result = ontology_tree.search_for_term(adding_dict["class_"]);
            if (ontological_result == None):
                ontology_tree.add_term(["unknown",adding_dict["class_"]]);
                adding_dict["category"] = "unknown";
            else:
                # So this will go [class, category, "Objs"];
                adding_dict["category"] = ontological_result[1];
                print(ontological_result);

            if (DEBUG):
                print("setting category to", adding_dict["category"]);

        return adding_dict, metadata;
    # ontology_tree.print_graph();

    def pickupable_callback(adding_dict:dict, obj_id:str):
        non_pickupable:list = ["table", "person"];
        if adding_dict["class_"] in non_pickupable:
            adding_dict["pickupable"] = False;
        else:
            adding_dict["pickupable"] = True;
        return adding_dict, obj_id;
    
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
        query_response=orion_actions.srv.SOMQueryObjectsResponse,
        input_parent=orion_actions.srv.SOMAddObject,
        input_response=orion_actions.srv.SOMAddObjectResponse
    );
    object_visualisation_manager:RvizVisualisationManager = RvizVisualisationManager(
        im_server=interactive_marker_server,
        colour_a=0.7, colour_r=0.0, colour_g=0.2, colour_b=1.0,
        class_attr="class_", size_attr="size", position_attr="obj_position"
    );
    object_manager:CollectionManager = CollectionManager(
        object_types,
        "objects",
        memory_manager=mem_manager,
        visualisation_manager=object_visualisation_manager,
        sort_queries_by="observation_batch_num"
    );
    def num_observation_threshold_query_callback(query_dict:dict):
        if 'num_observations' not in query_dict:
            query_dict['num_observations'] = {"$gt" : NUM_OBSERVATIONS_THRESHOLD_FOR_QUERY};
        else:
            threshold = query_dict['num_observations'];
            query_dict['num_observations'] = {"$gt" : threshold};
        return query_dict;
    object_manager.collection_query_callbacks.append(num_observation_threshold_query_callback);

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
        max_distance={"default":1, "person":3},
        class_identifier="class_",
        first_observed_attr="first_observed_at",
        last_observed_attr="last_observed_at",
        observed_at_attr="observed_at",
        observation_batch_num="observation_batch_num",
        last_observation_batch="last_observation_batch",
        positional_covariance_attr="covariance_mat",
        observation_counter_attr="num_observations",
        suppress_double_detections=False,
        suppression_distance_dict={'suppression_test_type':0.1}
    );
    observation_arg_name_defs.dont_transfer.append("covariance_mat");
    observation_arg_name_defs.dont_transfer.append("transform_cov_to_diagonal");
    # This is specifically for updating directly, and so we don't want this to be set to 
    # False upon every observation (although if the object's been moved, that shouldn't
    # actually be a problem).
    observation_arg_name_defs.dont_transfer.append("picked_up");
    observation_arg_name_defs.cross_ref_attr.append("class_");
    observation_manager:ConsistencyChecker = ConsistencyChecker(
        pushing_to=object_manager,
        types=observation_types,
        service_name="observations",
        consistency_args=observation_arg_name_defs,
        collection_input_callbacks=[ontology_observation_getCategory_callback, pickupable_callback]
    );


    # The input service has been completely rewritten for the region manager. 
    # The input_parent/input_response fields are set to None in the constructor.
    # Even so, it makes no sense to add them here as well.
    object_region_types = TypesCollection(
        base_ros_type=orion_actions.msg.SOMBoxRegion,
        query_parent=orion_actions.srv.SOMQueryRegions,
        query_response=orion_actions.srv.SOMQueryRegionsResponse
    );
    region_visualisation_manager:RvizVisualisationManager = RvizVisualisationManager(
        im_server=interactive_marker_server_regions,
        colour_a=0.7, colour_r=0.9, colour_g=0.2, colour_b=0.2,
        class_attr="name", size_attr="dimension", position_attr="corner_loc"
    );
    object_region_manager:RegionManager = RegionManager(
        memory_manager=mem_manager,
        types=object_region_types,
        service_name="object_regions",
        querying_within=object_manager,
        positional_parameter="obj_position",
        region_visualisation_manager=region_visualisation_manager
    );
    
    rospy.wait_for_service('/som/object_regions/basic_query');
    region_query_srv = rospy.ServiceProxy('/som/object_regions/basic_query', orion_actions.srv.SOMQueryRegions);
    region_query = orion_actions.srv.SOMQueryRegionsRequest();
    region_query.query.name = "arena_boundry";
    arena_boundary_regions:orion_actions.srv.SOMQueryRegionsResponse = region_query_srv(region_query);
    if len(arena_boundary_regions.returns) == 0:
        arena_boundary_region = orion_actions.msg.SOMBoxRegion();
    else:    
        arena_boundary_region:orion_actions.msg.SOMBoxRegion = arena_boundary_regions.returns[0] if len(arena_boundary_regions.returns) else None;
    
    def push_person_callback(adding:dict, metadata:dict):
        if len(metadata['obj_uid']) == 0:
            return adding, metadata;

        object_position = utils.dict_to_obj(adding["obj_position"], geometry_msgs.msg.Pose());

        if object_region_manager.point_in_region(arena_boundary_region, object_position.position) == False:
            return adding, metadata;

        if adding["class_"] == "person":
            human_query:list = human_manager.queryIntoCollection({"object_uid":metadata['obj_uid']});
            # So we want there to be one entry that's consistent with this object_uid.
            # Note that "object_uid" is what is being checked for consistency so
            # there should never be more than 1. 
            # NOTE: Maybe if True...
            # We might want it updating position all the time.
            if len(human_query) == 0:
                adding_human = orion_actions.msg.HumanObservation();
                adding_human.object_uid = metadata['obj_uid'];
                adding_human.obj_position = utils.dict_to_obj(adding["obj_position"], geometry_msgs.msg.Pose());
                adding_human.observed_at = utils.numericalTimeToROSTime(adding["observed_at"]);
                adding_human.spoken_to_state = orion_actions.msg.Human._NOT_SPOKEN_TO;
                adding_human.height = adding['size']['z'];
                human_observation_manager.addItemToCollectionDict(
                    utils.obj_to_dict(adding_human));

        return adding, metadata;

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