import numpy;
import utils;

from orion_actions.msg import Relation;

class RelationManager:
    def __init__(self, positional_attr:str):
        self.positional_attr = positional_attr;

    
    def get_relation_dict(self, cur_robot_pose:Pose, obj1:dict, obj2:dict) -> Relation:
        """
        HEAVILY inspired by Mark Richter's relational code from the old system.
        """

        dist_thr = 2;

        output_relation:Relation = Relation();
        robot_pos = utils.getPoint(utils.obj_to_dict(cur_robot_pose.position));
        obj_one_pos = utils.getPoint(obj1[self.positional_attr]);
        obj_two_pos = utils.getPoint(obj2[self.positional_attr]);

        print(robot_pos);
        print(obj_one_pos);
        print(obj_two_pos);

        robot_to_two = obj_two_pos - robot_pos
        two_to_one = obj_one_pos - obj_two_pos

        # if the distance between the two objects is greater than the threshold then none of the relations are true
        if numpy.linalg.norm(obj_one_pos - obj_two_pos) > dist_thr:
            output_relation.not_near = True
            return output_relation
        else:
            output_relation.near = True
        # near, not_near set.

        # vertical relation
        eps = 0.001 # use tolerance otherwise comparing relations of object to itself returns true for some fields
        if obj_one_pos[2] > obj_two_pos[2] + eps:
            output_relation.above = True
        elif obj_one_pos[2] < obj_two_pos[2] - eps:
            output_relation.below = True
        # near, not_near, above, below set.

        # forwards and backwards
        if numpy.dot(robot_to_two, two_to_one) > eps:
            output_relation.behind = True
        elif numpy.dot(robot_to_two, two_to_one) < -eps:
            output_relation.frontof = True
        # near, not_near, above, below, behind, frontof set.

        # left and right
        cross_pr = numpy.cross(robot_to_two, two_to_one)
        if cross_pr[2] > eps:
            output_relation.left = True
        elif cross_pr[2] < -eps:
            output_relation.right = True
        # near, not_near, above, below, behind, frontof, left, right set.
        
        # All that is now needed is left_most and right_most, 
        # but that won't happen here (because we're only looking at two objects, not
        # the whole set).

        return output_relation        


    pass;