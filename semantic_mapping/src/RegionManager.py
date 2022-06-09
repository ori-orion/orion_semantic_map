import numpy
import utils;
from CollectionManager import CollectionManager, TypesCollection;

import tf;
import geometry_msgs.msg;


class RegionManager(CollectionManager):
    """
    The whole aim here is to do introspection on one object to work out if it's
    within a cuiboidal region of corner location `corner_loc` and size `dimension`.

    """
    def __init__(
        self, 
        pushing_to:CollectionManager, 
        types:TypesCollection, 
        service_name:str,
        corner_loc:str,
        dimension:str):
        
        super(RegionManager, self).__init__(
            types=types, 
            service_name=service_name, 
            memory_manager=pushing_to.memory_manager);

        self.corner_loc = corner_loc;
        self.dimension = dimension;

        # self.transform_listener = tf.TransformListener();

        

    def point_in_region(self, region:dict, point:geometry_msgs.msg.Point) -> bool:
        transformed_pose = geometry_msgs.msg.Pose();
        transformed_pose:geometry_msgs.msg.Pose() = utils.dict_to_obj(region[self.corner_loc], transformed_pose);

        transformed_point = transformed_pose * point;
        print(transformed_point);
        pass;
    pass;