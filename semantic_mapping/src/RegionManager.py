"""
This is for querying within regions.
Regions are defined by the type SOMBoxRegion, this defining a cuboid in space that can be queried.
Each region is stored within the collection specified in the service name collection defined in main.py.
Each region is also given a static tf named [self.region_tf_prefix][collection_uid].
Regions are entities we typically want to last the entire competition, and so are given 
    SESSION_NUM = CollectionManager.PRIOR_SESSION_ID automatically. This will save them ad-infinitem (or until it's reset). Having a specific reset 
    call might be a plan tbh.
"""


import numpy
import utils;
from CollectionManager import CollectionManager, TypesCollection, SERVICE_ROOT;
from MemoryManager import DEBUG, MemoryManager, SESSION_ID;
from visualisation import RvizVisualisationManager;

from orion_actions.msg import SOMBoxRegion;

import orion_actions.msg
import orion_actions.srv

import rospy;
import genpy;
import tf2_ros;
import tf2_geometry_msgs;
import geometry_msgs.msg;

DEBUG_REGION_VISUALISATIONS:bool = True;

class RegionManager(CollectionManager):
    """
    The whole aim here is to do introspection on one object to work out if it's
    within a cuiboidal region of corner location `corner_loc` and size `dimension`.
    """
    def __init__(
        self, 
        memory_manager:MemoryManager, 
        types:TypesCollection, 
        service_name:str,
        querying_within:CollectionManager,
        positional_parameter:str,
        region_visualisation_manager:RvizVisualisationManager=None):
        
        # The method here is completely different! We therefore don't want the default
        # service to be created.
        types.input_parent = None;
        types.input_response = None;

        super(RegionManager, self).__init__(
            types=types, 
            service_name=service_name, 
            memory_manager=memory_manager);

        # self.corner_location = corner_location;
        # self.dimension = dimension;

        # Let's say we're looking for objects within a given region.
        # The region is defined by RegionManager. `querying_within`
        # defines the set within which we are looking. 
        self.querying_within:CollectionManager = querying_within;
        # We also need to know what the positional parameter within `querying_within` actually is.
        self.positional_parameter = positional_parameter;

        # Lists all the transforms that have been published. 
        self.static_transforms = [];
        # The static transform broadcaster itself.
        self.static_tb = tf2_ros.StaticTransformBroadcaster();

        self.tfBuffer = tf2_ros.Buffer();
        self.listener = tf2_ros.TransformListener(self.tfBuffer);

        self.global_frame = "map";

        # We don't want to blindly add transforms of random names into tf. This gives the prefix for these.
        self.region_tf_prefix = "region_";

        # We want every entry here to be a prior, so SESSION_ID = CollectionManager.PRIOR_SESSION_ID.
        # def session_num_to_prior_adding(adding_dict:dict, obj_uid:str):
        #     adding_dict[SESSION_ID] = CollectionManager.PRIOR_SESSION_ID;
        #     return adding_dict, obj_uid;
        # self.collection_input_callbacks.append(session_num_to_prior_adding);


        def session_num_to_prior_querying(query_dict:dict):
            query_dict[SESSION_ID] = -1;
            return query_dict;
        self.collection_query_callbacks.append(session_num_to_prior_querying);

        # So that we can implement separate logic to ensure it goes at the same location.
        # We aren't actually overriding because the inputs are completely different!
        # (Slight hack alert).
        self.region_visualisation_manager = region_visualisation_manager;

        self.setupROSServices();

    def transform_pt_to_global(self, transforming:geometry_msgs.msg.Point, region_tf_name:str) -> tf2_geometry_msgs.PointStamped:
        point_tf2 = tf2_geometry_msgs.PointStamped();
        point_tf2.point = transforming;        # NOTE: this is not actually the right type but this may well make no difference.
        point_tf2.header.stamp = rospy.Time.now();
        point_tf2.header.frame_id = self.global_frame;

        transformed_point:tf2_geometry_msgs.PointStamped = self.tfBuffer.transform(
            point_tf2, region_tf_name, rospy.Duration(1));

        # To prevent a race condition, we need to add a delay here!
        try:
            transformed_point:tf2_geometry_msgs.PointStamped = self.tfBuffer.transform(
                point_tf2, region_tf_name, rospy.Duration(1));
        except:
            transformed_point = tf2_geometry_msgs.PointStamped();
            rospy.logerr("transform raised an error!");

        return transformed_point;


    def create_region(self, transform:geometry_msgs.msg.TransformStamped, region_name:str, size:geometry_msgs.msg.Vector3):
        if DEBUG:
            print("Creating a region");

        adding = SOMBoxRegion();
        adding.corner_loc = transform;
        adding.dimension = size;
        adding.name = region_name;

        region_dict:dict = utils.obj_to_dict(adding);
        region_dict[SESSION_ID] = CollectionManager.PRIOR_SESSION_ID;
        region_id:str = str(self.addItemToCollectionDict(region_dict));

        self.publish_transform(transform, self.region_tf_prefix + region_id);

        if self.region_visualisation_manager != None:
            self.publish_visualisation_box(transform, region_name, size, region_id);
        
        return region_id;

    def publish_transform(self, transform:geometry_msgs.msg.TransformStamped, child_frame_id:str) -> None:
        transform.header.stamp = rospy.Time.now();
        transform.header.frame_id = self.global_frame;
        transform.child_frame_id = child_frame_id;

        self.static_transforms.append(transform);

        print("\tPublishing transform of name", child_frame_id);
        self.static_tb.sendTransform(self.static_transforms);

    
    def publish_visualisation_box(self, 
        transform:geometry_msgs.msg.TransformStamped, 
        region_name:str, 
        size:geometry_msgs.msg.Vector3,
        region_id:str):

        print("\tExecuting Region visualisation.");

        # This needs to be done because I believe visualisations are drawn with the point at their centre.
        half_size_point = geometry_msgs.msg.Point();
        half_size_point.x = size.x/2# + transform.transform.translation.x;
        half_size_point.y = size.y/2# + transform.transform.translation.y;
        half_size_point.z = size.z/2# + transform.transform.translation.z;
        # transformed_hf_size:tf2_geometry_msgs.PointStamped = self.transform_pt_to_global(half_size_point, self.region_tf_prefix + region_id);

        rotation_mat:numpy.matrix = utils.quaternion_to_rot_mat(transform.transform.rotation);
        rotated_corner_point = numpy.matmul(rotation_mat, numpy.asarray([size.x/2, size.y/2, size.z/2]));

        print("Half size:", half_size_point, " rotated to", rotated_corner_point);

        # transformed_hf_size then needs to be added to the corner loc to get the centre loc.
        centre_loc = geometry_msgs.msg.Pose();
        centre_loc.position.x = rotated_corner_point[0] + transform.transform.translation.x;
        centre_loc.position.y = rotated_corner_point[1] + transform.transform.translation.y;
        centre_loc.position.z = rotated_corner_point[2] + transform.transform.translation.z;

        # centre_loc.position.x = transform.transform.translation.x;
        # centre_loc.position.y = transform.transform.translation.y;
        # centre_loc.position.z = transform.transform.translation.z;
        centre_loc.orientation = transform.transform.rotation;

        self.region_visualisation_manager.add_object(
            id=self.region_tf_prefix + region_id,
            pose=centre_loc,
            size=size,
            obj_class=region_name);
        pass;


    def point_in_region(self, region:SOMBoxRegion, point:geometry_msgs.msg.Point) -> bool:
        """
        Returns whether the point is in the region.
        """
        transformed_point:tf2_geometry_msgs.PointStamped = self.transform_pt_to_global(point, self.region_tf_prefix + region.UID);

        # print(transformed_point);

        attr = ["x", "y", "z"];

        # Iterates over the parameters of {x,y,z}.
        for a in attr:
            dim = getattr(transformed_point.point, a);
            if dim < 0:
                return False;
            if dim > getattr(region.dimension, a):
                return False;

        return True;

    def queryRegionROSEntryPoint(self, query:orion_actions.srv.SOMRegionQueryRequest):
        """
        Here is the entry point. query has two fields:
            string region_name
            SOMObject query
        It should be noted that it is possible for multiple boxes to form a single region,
         and thus multiple regions could be returned per query for `region_name`. 
         We therefore want to iterate through all of these in our searching.
        """
        region_name = query.region_name;
        # All regions are priors...
        boxes = self.queryIntoCollection({
            "name":region_name,
            SESSION_ID: CollectionManager.PRIOR_SESSION_ID});

        # This is the query into the thing we're looking for the objects.
        # This should mirror CollectionManager.rosQueryEntrypoint(...).
        query_responses:list = self.querying_within.queryIntoCollection(
            utils.obj_to_dict(
                query.query, 
                ignore_default=True,
                ignore_of_type=[rospy.Time, rospy.Duration, genpy.rostime.Time],
                convert_caps=True));

        output = orion_actions.srv.SOMRegionQueryResponse();
        output_list = [];
        for query_response in query_responses:
            query_response:dict;
            if self.positional_parameter in query_response:
                if 'x' in query_response[self.positional_parameter]:
                    pos:geometry_msgs.msg.Point = utils.dict_to_obj(
                        query_response[self.positional_parameter], 
                        geometry_msgs.msg.Point());
                else:
                    pos:geometry_msgs.msg.Point = utils.dict_to_obj(
                        query_response[self.positional_parameter], 
                        geometry_msgs.msg.Pose()).position;

                for box in boxes:
                    # print(box);
                    box_som_msg:SOMBoxRegion = utils.dict_to_obj(box, SOMBoxRegion());
                    box_som_msg.UID = str(box[utils.PYMONGO_ID_SPECIFIER]);
                    # print(box_som_msg);

                    if self.point_in_region(box_som_msg, pos):
                        output_list.append(utils.dict_to_obj(query_response, orion_actions.msg.SOMObject()));

                        if self.querying_within.visualisation_manager != None and DEBUG_REGION_VISUALISATIONS:
                            self.querying_within.visualisation_manager.add_obj_dict(
                                query_response, 
                                str(query_response[utils.PYMONGO_ID_SPECIFIER]));


        output.returns = output_list;
        return output;


    def addRegionROSEntryPoint(self, adding:orion_actions.srv.SOMAddRegionRequest) -> orion_actions.srv.SOMAddRegionResponse:
        corner_loc_stamped = geometry_msgs.msg.TransformStamped();
        corner_loc_stamped.transform = adding.adding.corner_loc;
        uid:str = self.create_region(corner_loc_stamped, adding.adding.name, adding.adding.dimension);
        return orion_actions.srv.SOMAddRegionResponse(uid);


    def setupROSServices(self):
        rospy.Service(
            SERVICE_ROOT + self.service_name + "/region_query",
            orion_actions.srv.SOMRegionQuery,
            self.queryRegionROSEntryPoint);

        rospy.Service(
            SERVICE_ROOT + self.service_name + "/input",
            orion_actions.srv.SOMAddRegion,
            self.addRegionROSEntryPoint);
