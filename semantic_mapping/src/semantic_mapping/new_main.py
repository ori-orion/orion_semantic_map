#!/usr/bin/env python3


from interactive_markers.interactive_marker_server import InteractiveMarkerServer

from new_version.MemoryManager import MemoryManager;

import Ontology
from OntologySimilarityManager import OntologySimilarityManager
from new_version.ObjectsManager import ObjectsManager
from new_version.visualisation import RvizVisualisationManager;
import utils;

import rospy;
import numpy;
import tf2_ros;
from geometry_msgs.msg import Point
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
import new_version.profiler as prof

NUM_OBSERVATIONS_THRESHOLD_FOR_QUERY = 0;

class MemSys:
    def __init__(self):
        rospy.init_node('som_manager_new');

        self.mem_manager: MemoryManager = MemoryManager();        

        self.setupOntology();
        self.setupObjects();
        #self.setupRegions();


        #object_relational_manager = RelationManager(
        #    operating_on=self.object_manager,
        #    positional_attr="obj_position",
        #    service_base=orion_actions.srv.SOMRelObjQuery,
        #    service_response=orion_actions.srv.SOMRelObjQueryResponse,
        #    match_type=orion_actions.msg.Match
        #);

        rospy.loginfo("Memory systems set up!");


    def setupOntology(self):
        current_dir = os.path.dirname(__file__)
        
        self.ontology_tree = Ontology.read_file(current_dir + "/labels.txt");
        self.ontology_tree.print_graph();

        self.similarity_manager = OntologySimilarityManager(
            current_dir + '/taxonomyLabels.txt',
            'similarity')


    def setupObjects(self):
        interactive_marker_server = InteractiveMarkerServer("zzz_som/obj_vis")

        object_visualisation_manager: RvizVisualisationManager = RvizVisualisationManager(
            im_server=interactive_marker_server,
            colour_a=0.7, colour_r=0.0, colour_g=0.2, colour_b=1.0,
            class_attr="class_", size_attr="size", position_attr="obj_position"
        )

        self.object_manager: ObjectsManager = ObjectsManager(
            self.mem_manager, 
            self.ontology_tree, 
            visualisation_manager=object_visualisation_manager,
            collection_name="objects",
            sort_queries_by="last_observation_batch",
            max_distance_same_obj=0.,#{"default":1, "person":0.7},
            dont_edit_on_update=[])


    #def setupRegions(self):
    #    interactive_marker_server_regions = InteractiveMarkerServer("zzz_som/region_vis")#

    #    # The input service has been completely rewritten for the region manager. 
    #    # The input_parent/input_response fields are set to None in the constructor.
    #    # Even so, it makes no sense to add them here as well.
    #    object_region_types = TypesCollection(
    #        base_ros_type=orion_actions.msg.SOMBoxRegion,
    #        query_parent=orion_actions.srv.SOMQueryRegions,
    #        query_response=orion_actions.srv.SOMQueryRegionsResponse
    #    );
    #    region_visualisation_manager:RvizVisualisationManager = RvizVisualisationManager(
    #        im_server=interactive_marker_server_regions,
    #        colour_a=0.5, colour_r=0.2, colour_g=0.2, colour_b=0.9,
    #        class_attr="name", size_attr="dimension", position_attr="corner_loc"
    #    );
    #    self.object_region_manager:RegionManager = RegionManager(
    #        memory_manager=self.mem_manager,
    #        types=object_region_types,
    #        service_name="object_regions",
    #        querying_within=self.object_manager,
    #        positional_parameter="obj_position",
    #        region_visualisation_manager=region_visualisation_manager
    #    );


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

        self.add_to_collection_av_time = [0., 0.];
        self.tf_transformation_av_time = [0., 0.];
        self.num_items = [0, 0];

    # We essentially want to forward detections from orion_recognition over to the SOM database. This should do that.
    def forwardDetectionsToSOM(self, data:DetectionArray):
       
        printing = "";

        tf_header = data.header;
        tf_header.stamp = rospy.Time.now();
        tf_header.frame_id = self.camera_frame
        tf_list:List[geometry_msgs.msg.TransformStamped] = [];
        tf_name_list = [];


        for detection in data.detections: # type:ignore
            detection:Detection; 
            if detection.label.confidence < 0.5:
                continue;
            
            forwarding = SOMObservation();

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

            idx = 1 if forwarding.class_ == "person" else 0
            self.num_items[idx] += 1;

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
            prof.start("compute-global-pose")
            try:
                p_global_frame:tf2_geometry_msgs.PoseStamped = self.tfBuffer.transform(
                    obj_point_2, self.global_frame, timeout=rospy.Duration(secs=1));
            except:
                try:
                    obj_point_2.header.stamp = rospy.Time.now();
                    p_global_frame:tf2_geometry_msgs.PoseStamped = self.tfBuffer.transform(
                        obj_point_2, self.global_frame, timeout=rospy.Duration(secs=1));
                except:
                    p_global_frame = tf2_geometry_msgs.PoseStamped();
                    rospy.logerr("new_main.py: transform raised an error!");
                    return;
            prof.record("compute-global-pose")
            toc = time.perf_counter();
            self.tf_transformation_av_time[idx] += toc-tic;    
    
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
            uncertainty_rot = utils.quaternion_to_rot_mat(camera_to_global.transform.rotation);
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
            prof.start("add-obs-callback")
            mem_sys.object_manager.addObservationToCollection(forwarding);
            prof.record("add-obs-callback")
            toc = time.perf_counter();
            self.add_to_collection_av_time[idx] += toc-tic;
        
            # Dealing with the publishing of tfs. Based on orion_recognition/.../detection_tf_publisher.py
            individual_tf = geometry_msgs.msg.TransformStamped();
            individual_tf.header = tf_header;
            tf_name = copy.copy(mem_sys.object_manager.latest_tf_name) # type:ignore
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

        self.transform_broadcaster.sendTransform(tf_list);
        #if self.num_items[0] > 0:# and self.num_items[1] > 0:
        #    print("Average times:");
        #    print("\tAvg over:", self.num_items)
        #    print("\tGetting global pose: ", self.tf_transformation_av_time[0]/self.num_items[0])#, self.tf_transformation_av_time[1]/self.num_items[1]);
        #    print("\tAdding to collection:", self.add_to_collection_av_time[0]/self.num_items[0])#, self.add_to_collection_av_time[1]/self.num_items[1]);
        #    print(printing + "--------------------------------")
        prof.print_description()
        self.batch_num += 1;





if __name__ == '__main__':
    mem_sys = MemSys();

    detect_to_observe = DetectToObserve(mem_sys);

    rospy.spin();
