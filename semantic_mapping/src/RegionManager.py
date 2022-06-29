import numpy
import utils;
from CollectionManager import CollectionManager, TypesCollection;
from MemoryManager import MemoryManager;

from orion_actions.msg import SOMBoxRegion;
import orion_actions.msg

import rospy;
import genpy;
import tf2_ros;
import tf2_geometry_msgs;
import geometry_msgs.msg;

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
        positional_parameter:str):
        
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

        self.static_tb = tf2_ros.StaticTransformBroadcaster();
        self.publisher:rospy.Publisher = self.static_tb.pub_tf;

        self.tfBuffer = tf2_ros.Buffer();
        self.listener = tf2_ros.TransformListener(self.tfBuffer);

        self.global_frame = "map";

        # We don't want to blindly add transforms of random names into tf. This gives the prefix for these.
        self.region_tf_prefix = "region_";

        # Beginning to test stuff...
        transform_in = geometry_msgs.msg.TransformStamped();
        transform_in.transform.translation = geometry_msgs.msg.Vector3(1,0,0);
        transform_in.transform.rotation = geometry_msgs.msg.Quaternion(0,0,0,1);

        self.publish_transform(transform_in, "region_1");

        # self.point_in_region("region_1", point_in);


    def create_region(self, transform:geometry_msgs.msg.TransformStamped, region_name:str, size:geometry_msgs.msg.Vector3):
        self.publish_transform(transform, self.region_tf_prefix + region_name);

        adding = SOMBoxRegion();
        adding.corner_loc = transform;
        adding.dimension = size;
        adding.name = region_name;

        self.addItemToCollectionDict(utils.obj_to_dict(adding));
        
    def publish_transform(self, transform:geometry_msgs.msg.TransformStamped, child_frame_id:str) -> None:

        transform.header.stamp = rospy.Time.now();
        transform.header.frame_id = self.global_frame;
        transform.child_frame_id = child_frame_id;

        self.static_tb.sendTransform(transform);


    def point_in_region(self, region:SOMBoxRegion, point:geometry_msgs.msg.Point) -> bool:
        
        point_tf2 = tf2_geometry_msgs.PointStamped();
        point_tf2.point = point;
        point_tf2.header.stamp = rospy.Time.now();
        point_tf2.header.frame_id = self.global_frame;

        try:
            transformed_point:tf2_geometry_msgs.PointStamped = self.tfBuffer.transform(
                point_tf2, region.UID);
        except:
            transformed_point = tf2_geometry_msgs.PointStamped();
            rospy.logerr("transform raised an error!");

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
        boxes = self.queryIntoCollection({"region_name":region_name});

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
            if self.positional_parameter in query_response:
                if 'x' in query_response[self.positional_parameter]:
                    pos:geometry_msgs.msg.Point = utils.dict_to_obj(query_response[self.positional_parameter], geometry_msgs.msg.Point());
                else:
                    pos:geometry_msgs.msg.Point = utils.dict_to_obj(query_response[self.positional_parameter], geometry_msgs.msg.Pose()).position;

                for box in boxes:
                    box_som_msg:SOMBoxRegion = utils.dict_to_obj(box, SOMBoxRegion());

                    if self.point_in_region(box_som_msg, pos):
                        output_list.append(utils.dict_to_obj(query_response, orion_actions.msg.SOMObject()));

                pass;
            pass;
        output.returns = output_list;
        return output;