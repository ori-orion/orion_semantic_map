"""
Defines the infrastructure around getting the relation between objects.
"""
import numpy;
import utils;
import rospy;
import genpy;

from orion_actions.msg import Relation;
from geometry_msgs.msg import Pose, Point;

from CollectionManager import CollectionManager, SERVICE_ROOT;

class RelationManager:
    """
    positional_attr     - The attribute name for the position of an
                            object within the collection managed by operating_on.
    service_base        - The base type for the service call.
    service_response    - The return type for the service call.
    operating_on        - The wrapper class for the mongodb collection we're operating on. 

    Required fields within input field for the relational services.
        - obj1                  - The first object
        - obj2                  - The second object
        - relation              - The relational query between obj1 and obj2.
                                    (Note that we're looking for matches to [obj] [relation] [obj2])
        - current_robot_pose    - The goemetry_msgs/Pose of the robot (so that we can work out 
                                    left/right and behind/infront matches.)
    Note that there should be a single output field of an array of matches (those being defined with the following fields):
        - obj1
        - obj2
        - relation
        where everything is defined as before.
    There is no naming constraint on this last outermost variable, it being the only one necessary for the output.
    """
    def __init__(self, operating_on:CollectionManager, positional_attr:str, service_base:type, service_response:type, match_type:type):
        self.positional_attr = positional_attr;

        self.service_base:type = service_base;
        self.match_type:type = match_type;
        self.service_response:type = service_response;

        self.operating_on = operating_on;

        self.setup_ROS_services();


    def ROS_perform_relational_query(self, input):
        obj1_dict = utils.obj_to_dict(
            input.obj1, 
            ignore_default=True,
            ignore_of_type=[rospy.Time, rospy.Duration, genpy.rostime.Time]);
        obj2_dict = utils.obj_to_dict(
            input.obj2, 
            ignore_default=True,
            ignore_of_type=[rospy.Time, rospy.Duration, genpy.rostime.Time]);
        
        return self.perform_relational_query(obj1_dict, obj2_dict, input.relation, input.current_robot_pose);


    def perform_relational_query(self, obj1:dict, obj2:dict, relation:Relation, cur_robot_pose:Pose):
        obj1_query_result:list = self.operating_on.queryIntoCollection(obj1);
        obj2_query_result:list = self.operating_on.queryIntoCollection(obj2);

        querying_relation_dict = utils.obj_to_dict(relation);

        # We only want to look at outputs for which the relations match up.
        # I.e., see the logic below. 
        def compare_relational_dicts(comparing_with:dict) -> bool:
            output = True
            for key in querying_relation_dict.keys():
                if querying_relation_dict[key] == True and comparing_with[key] == False:
                    return False;
            return output;

        matches = [];
        for o1 in obj1_query_result:
            for o2 in obj2_query_result:
                relation_out:Relation = self.get_relation_dict(cur_robot_pose, o1, o2);
                relation_out_dict = utils.obj_to_dict(relation_out);

                if compare_relational_dicts(relation_out_dict):
                    match_appending = self.match_type();                    
                    match_appending.obj1 = utils.dict_to_obj(o1, match_appending.obj1);
                    match_appending.obj2 = utils.dict_to_obj(o2, match_appending.obj2);
                    match_appending.relation = relation_out;

                    matches.append(match_appending);
        
        output = self.service_response();
        setattr(output, utils.get_attributes(output)[0], matches);
        return output;        

    
    def get_relation_dict(self, cur_robot_pose:Pose, obj1:dict, obj2:dict) -> Relation:
        """
        HEAVILY inspired by Mark Richter's relational code from the old system.
        """

        dist_thr = 2;

        output_relation:Relation = Relation();
        robot_pos = utils.getPoint(utils.obj_to_dict(cur_robot_pose.position));
        obj_one_pos = utils.getPoint(obj1[self.positional_attr]);
        obj_two_pos = utils.getPoint(obj2[self.positional_attr]);

        # print(robot_pos);
        # print(obj_one_pos);
        # print(obj_two_pos);

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


    def setup_ROS_services(self):

        rospy.Service(
            SERVICE_ROOT + self.operating_on.service_name + '/relational_query', 
            self.service_base, 
            self.ROS_perform_relational_query);