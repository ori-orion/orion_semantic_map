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
import Ontology
from OntologySimilarityManager import OntologySimilarityManager
from visualisation import RvizVisualisationManager;
import utils;

import rospy;
import numpy;
import tf2_ros;
from geometry_msgs.msg import PoseStamped, Point, Pose
import tf2_geometry_msgs

import orion_actions.msg
import orion_actions.srv
import geometry_msgs.msg;
from orion_actions.msg import *;
from orion_actions.srv import *;

import os;
import math;
import copy;
import time;

from typing import Dict, List

# import semantic_mapping.srv
# import semantic_mapping.msg

# print(__file__);


NUM_OBSERVATIONS_THRESHOLD_FOR_QUERY = 0;

class MemSys:
    def __init__(self):
        rospy.init_node('som_manager');

        self.mem_manager:MemoryManager = MemoryManager();        

        self.setupOntology();
        self.setupHumans();
        self.setupObjects();
        self.setupRegions();

        # This needs to be here because we need the callback to be called AFTER `obj_uid`
        # has been assigned by the observation_manager. Thus the Observation manager 
        # needs to be created first.
        self.observation_manager.collection_input_callbacks.append(self.push_person_callback);

        object_relational_manager:RelationManager = RelationManager(
            operating_on=self.object_manager,
            positional_attr="obj_position",
            service_base=orion_actions.srv.SOMRelObjQuery,
            service_response=orion_actions.srv.SOMRelObjQueryResponse,
            match_type=orion_actions.msg.Match
        );

        rospy.loginfo("Memory systems set up!");


    def setupOntology(self):
        current_dir = os.path.dirname(__file__)
        
        self.ontology_tree = Ontology.read_file(current_dir + "/labels.txt");
        self.ontology_tree.print_graph();

        self.similarity_manager = OntologySimilarityManager(
            current_dir + '/taxonomyLabels.txt',
            'similarity')

    # This will be a callback within observations for assigning the category of an object.
    def ontology_observation_getCategory_callback(self, adding_dict:dict, metadata:dict):
        # print("Ontology callback");
        if (len(adding_dict["category"]) == 0):
            ontological_result = self.ontology_tree.search_for_term(adding_dict["class_"]);
            if (ontological_result == None):
                self.ontology_tree.add_term(["unknown",adding_dict["class_"]]);
                adding_dict["category"] = "unknown";
            else:
                # So this will go [class, category, "Objs"];
                adding_dict["category"] = ontological_result[1];
                # print(ontological_result);

            if (DEBUG):
                print("setting category to", adding_dict["category"]);

        return adding_dict, metadata;

    # This will set the flag for whether something is pickupable or not.
    def pickupable_callback(self, adding_dict:dict, metadata:dict):
        # print("Pickupable callback");
        non_pickupable:list = ["table", "person"];
        if adding_dict["class_"] in non_pickupable:
            adding_dict["pickupable"] = False;
        else:
            adding_dict["pickupable"] = True;
        return adding_dict, metadata;

    # Pushing persons to the human collection.
    def push_person_callback(self, adding:dict, metadata:dict):
        # print("Push person callback");
        if len(metadata['obj_uid']) == 0:
            return adding, metadata;

        if adding["class_"] == "person":
            human_query:list = self.human_manager.queryIntoCollection({"object_uid":metadata['obj_uid']});
            # So we want there to be one entry that's consistent with this object_uid.
            # Note that "object_uid" is what is being checked for consistency so
            # there should never be more than 1. 
            # NOTE: Maybe if True...
            # We might want it updating position all the time.

            # Deprecated.
            # if len(human_query) == 0:
            #     adding_human = orion_actions.msg.HumanObservation();
            #     adding_human.object_uid = metadata['obj_uid'];
            #     adding_human.obj_position = utils.dict_to_obj(adding["obj_position"], geometry_msgs.msg.Pose());
            #     adding_human.observed_at = utils.dict_to_obj(adding["observed_at"], rospy.Time());
            #     adding_human.spoken_to_state = orion_actions.msg.Human._NOT_SPOKEN_TO;
            #     adding_human.height = adding['size']['z'];
            #     self.human_observation_manager.addItemToCollectionDict(
            #         utils.obj_to_dict(adding_human));

            human_obs = orion_actions.msg.HumanObservation();
            human_obs.object_uid = metadata['obj_uid'];
            human_obs.obj_position = utils.dict_to_obj(adding["obj_position"], geometry_msgs.msg.Pose());
            human_obs.observed_at = utils.dict_to_obj(adding["observed_at"], rospy.Time());
            # print(adding);
            if "last_observation_batch" in adding:
                human_obs.observation_batch_num = adding["last_observation_batch"];
            else:
                human_obs.observation_batch_num = adding["observation_batch_num"];
            
            if len(human_query) == 0:
                human_obs.spoken_to_state = orion_actions.msg.Human._NOT_SPOKEN_TO;
                human_obs.height = adding['size']['z'];
            else:
                # human_obs. = str(human_query[0][utils.PYMONGO_ID_SPECIFIER]);
                pass;
            self.human_observation_manager.addItemToCollection(human_obs);
            
        return adding, metadata;


    def setupHumans(self):
        # The human stuff needs to be before the object stuff if we are to push
        # humans as a result of seeing objects.
        self.human_types:TypesCollection = TypesCollection(
            base_ros_type=orion_actions.msg.Human,
            query_parent=orion_actions.srv.SOMQueryHumans,
            query_response=orion_actions.srv.SOMQueryHumansResponse
        );
        self.human_manager:CollectionManager = CollectionManager(
            types=self.human_types,
            service_name="humans",
            memory_manager=self.mem_manager
        );

        self.human_observation_types:TypesCollection = TypesCollection(
            base_ros_type=orion_actions.msg.HumanObservation,
            input_parent=orion_actions.srv.SOMAddHumanObs,
            input_response=orion_actions.srv.SOMAddHumanObsResponse
        );
        self.human_observation_manager_args:ConsistencyArgs = ConsistencyArgs(
            position_attr="obj_position",
            first_observed_attr="first_observed_at",
            last_observed_attr="last_observed_at",
            observed_at_attr="observed_at"
        );
        self.human_observation_manager_args.use_running_average_position = False;
        # human_observation_manager_args.cross_ref_attr.append("task_role");
        self.human_observation_manager_args.cross_ref_attr.append("object_uid");
        self.human_observation_manager:ConsistencyChecker = ConsistencyChecker(
            pushing_to=self.human_manager,
            types=self.human_observation_types,
            service_name="human_observations",
            consistency_args=self.human_observation_manager_args
        );


    def setupObjects(self):
        interactive_marker_server = InteractiveMarkerServer("zzz_som/obj_vis")

        self.object_types:TypesCollection = TypesCollection(
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
        self.object_manager:CollectionManager = CollectionManager(
            self.object_types,
            "objects",
            memory_manager=self.mem_manager,
            visualisation_manager=object_visualisation_manager,
            sort_queries_by="observation_batch_num"
        );
        def num_observation_threshold_query_callback(query_dict:dict, metadata:dict):
            if 'num_observations' not in query_dict:
                query_dict['num_observations'] = {"$gt" : NUM_OBSERVATIONS_THRESHOLD_FOR_QUERY};
            else:
                threshold = query_dict['num_observations'];
                query_dict['num_observations'] = {"$gt" : threshold};
            return query_dict, metadata;
        self.object_manager.collection_query_callbacks.append(num_observation_threshold_query_callback);

        # For assigning the tf names.
        self.latest_tf_index:Dict[str,int] = {};
        def assign_tf_name_input_callback(adding:dict, metadata:dict):
            # print("Tf name callback");
            if adding["class_"] in self.latest_tf_index:
                self.latest_tf_index[adding["class_"]] += 1;
            else:
                self.latest_tf_index[adding["class_"]] = 0;
            # print(adding);
            tf_name = adding["class_"] + "_" + str(self.latest_tf_index[adding["class_"]]);
            adding["tf_name"] = tf_name;
            metadata["tf_name"] = tf_name;
            return adding, metadata;
        self.object_manager.collection_input_callbacks.append(assign_tf_name_input_callback);

        self.observation_types:TypesCollection = TypesCollection(
            base_ros_type=orion_actions.msg.SOMObservation,
            input_parent=orion_actions.srv.SOMAddObservation,
            input_response=orion_actions.srv.SOMAddObservationResponse,
            query_parent=orion_actions.srv.SOMQueryObservations,
            query_response=orion_actions.srv.SOMQueryObservationsResponse
        );
        self.observation_arg_name_defs:ConsistencyArgs = ConsistencyArgs(
            position_attr="obj_position",
            size_attr="size",
            max_distance={"default":1, "person":0.7},
            class_identifier="class_",
            first_observed_attr="first_observed_at",
            last_observed_attr="last_observed_at",
            observed_at_attr="observed_at",
            observation_batch_num="observation_batch_num",
            last_observation_batch="last_observation_batch",
            # positional_covariance_attr="covariance_mat",
            observation_counter_attr="num_observations",
            suppress_double_detections=False,       # Currently the suppression of double detections if off!
            suppression_distance_dict={'suppression_test_type':0.1, 'person':0.5},
            tf_name_attr="tf_name",
            use_running_average_position=False
        );
        self.observation_arg_name_defs.dont_transfer.append("covariance_mat");
        self.observation_arg_name_defs.dont_transfer.append("transform_cov_to_diagonal");
        # This is specifically for updating directly, and so we don't want this to be set to 
        # False upon every observation (although if the object's been moved, that shouldn't
        # actually be a problem).
        self.observation_arg_name_defs.dont_transfer.append("picked_up");
        self.observation_arg_name_defs.cross_ref_attr.append("class_");
        self.observation_manager:ConsistencyChecker = ConsistencyChecker(
            pushing_to=self.object_manager,
            types=self.observation_types,
            service_name="observations",
            consistency_args=self.observation_arg_name_defs,
            collection_input_callbacks=[self.ontology_observation_getCategory_callback, self.pickupable_callback]
        );


    def setupRegions(self):
        interactive_marker_server_regions = InteractiveMarkerServer("zzz_som/region_vis")

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
        self.object_region_manager:RegionManager = RegionManager(
            memory_manager=self.mem_manager,
            types=object_region_types,
            service_name="object_regions",
            querying_within=self.object_manager,
            positional_parameter="obj_position",
            region_visualisation_manager=region_visualisation_manager
        );


class DetectToObserve:
    def __init__(self, mem_sys:MemSys):
        queue_size = 1;
        
        # Data about transform frames
        self.camera_frame = "head_rgbd_sensor_rgb_frame"
        self.global_frame = "map"

        self.tfBuffer = tf2_ros.Buffer();
        self.listener = tf2_ros.TransformListener(self.tfBuffer);
        # We are also going to want it to publish tf information so that we get consistent tfs per object.
        self.transform_broadcaster = tf2_ros.TransformBroadcaster();

        # Initialising the SOM services
        # rospy.wait_for_service('som/observe')
        # rospy.wait_for_service('som/query')
        rospy.wait_for_service('som/observations/input');
        self.observe_obj_srv = rospy.ServiceProxy(
            'som/observations/input', 
            SOMAddObservation);
        # self.observe_objs_srv = rospy.ServiceProxy('som/observe', SOMObserve)
        # self.query_objs_srv = rospy.ServiceProxy('som/query', SOMQuery)

        # The format of this is:
        # { object_type: forwarding }
        # where forwarding is of the type SOMObservation[] and is what gets sent across som/observe
        # This feels more foolproof than doing a query (as well as potentially being more time efficient) 
        # self.previous_detections = {};

        self.mem_sys = mem_sys;

        rospy.Subscriber('/vision/bbox_detections', DetectionArray, self.forwardDetectionsToSOM, queue_size=queue_size)

        self.batch_num = 1;        
        pass;

    # We essentially want to forward detections from orion_recognition over to the SOM database. This should do that.
    def forwardDetectionsToSOM(self, data:DetectionArray):
        printing = "";

        tf_header = data.header;
        tf_header.stamp = rospy.Time.now();
        tf_list:List[geometry_msgs.msg.TransformStamped] = [];
        tf_name_list = [];

        add_to_collection_av_time = 0;
        tf_transformation_av_time = 0;
        num_items = 0;

        for detection in data.detections:
            detection:Detection;
            num_items += 1;

            if detection.label.confidence < 0.5:
                continue;
            
            forwarding = SOMObservation();
            # print(dir(forwarding))
            # print(dir(forwarding.robot_pose));

            detection.label.name = detection.label.name.lower();

            # NOTE: Assuming SOMObservation.type is for the name of the object. This is most likely wrong!
            if "/" in detection.label.name:
                split_name = detection.label.name.split('/');
                forwarding.class_ = split_name[len(split_name)-1];
                forwarding.category = split_name[len(split_name)-2];
            else:
                forwarding.class_ = detection.label.name;
            forwarding.observation_batch_num = self.batch_num;
            forwarding.size = detection.size;
            # forwarding.timestamp = rospy.Time().now()

            # Getting the robot pose:
            camera_to_global:tf2_ros.TransformStamped = self.tfBuffer.lookup_transform(
                self.camera_frame, 
                self.global_frame, 
                rospy.Time());
            
            # forwarding.robot_pose = Pose();
            forwarding.camera_pose.position.x = camera_to_global.transform.translation.x;
            forwarding.camera_pose.position.y = camera_to_global.transform.translation.y;
            forwarding.camera_pose.position.z = camera_to_global.transform.translation.z;
            forwarding.camera_pose.orientation.w = camera_to_global.transform.rotation.w;
            forwarding.camera_pose.orientation.x = camera_to_global.transform.rotation.x;
            forwarding.camera_pose.orientation.y = camera_to_global.transform.rotation.y;
            forwarding.camera_pose.orientation.z = camera_to_global.transform.rotation.z;

            # Getting the position of the object in 3D space relative to the global frame.
            obj_point_2 = tf2_geometry_msgs.PoseStamped();
            obj_point_2.header.frame_id = self.camera_frame;
            obj_point_2.header.stamp = detection.timestamp;
            obj_point_2.pose.position = Point(
                detection.translation_x, detection.translation_y, detection.translation_z+forwarding.size.z/2);
            
            # object_point = PoseStamped()
            # object_point.header.frame_id = self.camera_frame;
            # object_point.header.stamp = detection.timestamp;
            # object_point.pose.position = Point(
            #     detection.translation_x, detection.translation_y, detection.translation_z);
            # transformed_stamped = self.tfBuffer.transform(
            #     object_point, self.global_frame, timeout=rospy.Duration(1));
            
            tic = time.perf_counter();

            try:
                p_global_frame:tf2_geometry_msgs.PoseStamped = self.tfBuffer.transform(
                    obj_point_2, self.global_frame, timeout=rospy.Duration(0.5));
            except:
                try:
                    obj_point_2.header.stamp = rospy.Time.now();
                    p_global_frame:tf2_geometry_msgs.PoseStamped = self.tfBuffer.transform(
                        obj_point_2, self.global_frame, timeout=rospy.Duration(0.5));
                except:
                    p_global_frame = tf2_geometry_msgs.PoseStamped();
                    rospy.logerr("detections_to_observations.py: transform raised an error!");
                    return;
            toc = time.perf_counter();
            tf_transformation_av_time += toc-tic;    
    
            # transformed_obj_point:PoseStamped = p_global_frame;
            forwarding.obj_position = p_global_frame.pose;
            forwarding.observed_at = rospy.Time.now();
            forwarding.colour = detection.color;

            # forwarding.header = std_msgs.msg.Header();
            # forwarding.header.stamp = rospy.Time.now();
            # NOTE need to check frame ID in the header. (Could that be that of the camera?)
            # forwarding.header.frame_id = self.global_frame;#.encode("ascii", "ignore");                

            # Setting up the covariance stuff.
            # Note that the covariance matrix is symmetric, so the order doesn't matter.
            # Note also that the parameters here could probably be adjusted!
            uncertainty_rot:numpy.matrix = utils.quaternion_to_rot_mat(camera_to_global.transform.rotation);
            cov_mat = numpy.matrix([[3,0,0],[0,0.5,0],[0,0,0.5]]);
            cov_transformed = numpy.matmul(uncertainty_rot.transpose(), numpy.matmul(cov_mat, uncertainty_rot));
            cov_t_linear = [];
            cov_transform_linear = [];
            for i in range(3):
                for j in range(3):
                    cov_t_linear.append(cov_transformed[i,j]);
                    cov_transform_linear.append(uncertainty_rot[i,j]);
            forwarding.covariance_mat = cov_t_linear;
            forwarding.transform_cov_to_diagonal = cov_transform_linear;
            
            # service_output:SOMObserveResponse = self.observe_obj_srv(forwarding);
            printing += "\t" + forwarding.class_ + "\n";

            tic = time.perf_counter();
            mem_sys.observation_manager.addItemToCollection(forwarding);
            toc = time.perf_counter();
            add_to_collection_av_time += toc-tic;
        
            # Dealing with the publishing of tfs. Based on orion_recognition/.../detection_tf_publisher.py
            individual_tf = geometry_msgs.msg.TransformStamped();
            individual_tf.header = tf_header;
            # print(mem_sys.object_manager.metadata_latent_adding);
            tf_name = copy.copy(mem_sys.object_manager.metadata_latent_adding["tf_name"]);
            individual_tf.child_frame_id = tf_name;
            individual_tf.transform.translation.x = detection.translation_x;
            individual_tf.transform.translation.y = detection.translation_y;
            individual_tf.transform.translation.z = detection.translation_z;
            individual_tf.transform.rotation.z = math.sin( -math.pi / 4 );
            individual_tf.transform.rotation.w = math.cos( -math.pi / 4 );
            tf_list.append(individual_tf);
            tf_name_list.append(tf_name);

            # addition_successful = service_output.obj_id;
            # obj_id_returned = service_output.obj_id;

            # print(obj_id_returned);
        # print(mem_sys.latest_tf_index);
        print(tf_name_list);
        self.transform_broadcaster.sendTransform(tf_list);
        print("Average times:");
        print("\tGetting global pose: ", tf_transformation_av_time/num_items);
        print("\tAdding to collection:", add_to_collection_av_time/num_items);
        print(printing + "--------------------------------")
        self.batch_num += 1;
        # print(mem_sys.object_manager.collection_input_callbacks);
        # print(mem_sys.observation_manager.collection_input_callbacks);





if __name__ == '__main__':
    mem_sys = MemSys();

    detect_to_observe = DetectToObserve(mem_sys);

    rospy.spin();
