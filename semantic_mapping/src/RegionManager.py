import numpy
import utils;
from CollectionManager import CollectionManager, TypesCollection;
from MemoryManager import MemoryManager;

import tf;
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
        corner_location:str,
        dimension:str):
        
        super(RegionManager, self).__init__(
            types=types, 
            service_name=service_name, 
            memory_manager=memory_manager);

        self.corner_location = corner_location;
        self.dimension = dimension;

        # self.transform_listener = tf.TransformListener();

        pose_in = geometry_msgs.msg.Pose();
        pose_in_dict = utils.obj_to_dict(pose_in);
        region_in = {self.corner_location:pose_in_dict};
        point_in = geometry_msgs.msg.Point();
        point_in.x = 1;

        print(pose_in);
        print(point_in);
        self.point_in_region(region_in, point_in);



    def point_in_region(self, region:dict, point:geometry_msgs.msg.Point) -> bool:
        transformed_pose = geometry_msgs.msg.Pose();
        transformed_pose:geometry_msgs.msg.Pose() = utils.dict_to_obj(region[self.corner_location], transformed_pose);

        transformed_point = transformed_pose * point;
        print(transformed_point);
        pass;
    pass;