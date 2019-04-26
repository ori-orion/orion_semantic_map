from .constants import Relation
import numpy as np
from som_object import InSOMObject

def _get_all_relations():
    """Returns a list of all possible Relation enum values."""
    return [Relation.LEFT,
    		Relation.RIGHT,
    		Relation.ABOVE,
    		Relation.BELOW,
    		Relation.AT,
    		Relation.ONTOP]


def query(som_template_one, relation, som_template_two, cur_robot_pose, mongo_object_store):
    """
    Performs a query to the semantic object database. This function uses
    SOMObject objects as templates. Meaning that any fields specified in
    the SOMObject we will require to be matched exactly, and any missing
    fields can be filled with any value.

    The query will return valid tuples (o1, ~, o2) which satisfy
    that o1 and o2 match the templates 'som_template_one' and
    'som_template_two', and that o1 ~ o2 is satisfied.

    Example queries
        query(x=milk_instance, tilde=Relation.BELOW, y=None)
            -> [(milk_instance, Relation.BELOW, coke_soda_instance),
                (milk_instance, Relation.BELOW, fanta_soda_instance),
                (milk_instance, Relation.BELOW, cheese_instance)]
        query(x=None, tilde=Relation.ONTOP, y=None)
            -> [(table_instance, Relation.ONTOP, coke_soda_instance),
                (table_instance, Relation.ONTOP, food_plate_instance),
                (shelf_instance, Relation.ONTOP, water_bottle_instance)]
        query(x=michael_instance, tilde=None, y=table_instance)
            -> [(michael_instance, Relation.AT, table_instance)]
        query(x=table_instance, tilde=None, y=michael_instance)
            -> []

    Args:
        som_obj_one: A SOMObservation to be used as a template to look up
            objects in the mongodb.
        relation: A Relation enum type specifying the relation
            required between objects, or, None, if we want to consider
            all possible relations.
        som_obj_two: A SOMObservation to be used as a template to look up
            objects in the mongodb.
        cur_robot_pos: The pose of the current robot, used to compute
            spatial relations. (These change depending on where the
            robot is facing).
        mongo_object_store: ...

    Returns:
        All valid (o1, ~, o2) tuples, for which o1 ~ o2 is true and
        o1 matches the template of 'som_obj_one' and o2 matches the
        template of 'som_obj_two'.
    """
    o1_matches = _mongo_som_objects_matching_template(som_template_one, mongo_object_store)
    o2_matches = _mongo_som_objects_matching_template(som_template_two, mongo_object_store)
    relations = [relation] if relation is not None else _get_all_relations()

    tuples = []
    for o1 in o1_matches:
        for o2 in o2_matches:
            rel = spatial_relation(cur_robot_pose, o1, o2)
            if rel in relations:
                matching_tuple = (o1, rel, o2)
                tuples.append(matching_tuple)

    return tuples

def _mongo_som_objects_matching_template(som_obs, mongo_object_store):
    """
    Treats the SOMObservation 'som_obs' as a template to match in mongo db.
    This returns a list of SOMObjects in the database matching the
    template.

    Args:
        som_obj: The SOMObservation (template) specified to make a query
        mongo_object_store: A mongodb store for SomObject instances
    Returns:
        A list of objects matching the template.
    """
    # Get queries for the mongo store (easiest way is currently to convert to 
    # InSOMObject and back)
    som_obj = InSOMObject.from_som_observation_message(som_obs)
    query_dict = som_obj.to_som_object_mongo_db_query()

    # Perform the query (return a list of SOMObject's)
    result = mongo_object_store.query(SOMObject._type, message_query=query_dict)
    return result

def _spatial_relation(cur_robot_pose, som_obj_one, som_obj_two):
    """
    This should return the relation ~ that makes the statement
    "som_obj_one ~ som_obj_two" true.

    We assume that only one spatial relation can be true at a time?

    This can likely be computed using dot products, and checking which
    side of a given plane we are. So we say define 6/8 planes, and
    depending on the signs of dot products, we give a spatial relation.

    One complexity is how to consider the shape of the object. For
    example, the pose of a table may be the central point of the table.
    So if an object is on the left of the table, it's not clear how to
    return that the object is "ontop" the table, rather than to the
    "left" of. I guess object shapes should be taken into account? Or
    we try to make sure that the poses are at the center of mass?

    Args:
        cur_robot_pose: The current pose of the robot, used as a
                        reference frame to work out spatial relations.
        som_obj_one: The first InSOMObject to consider the relation for.
        som_obj_two: The second InSOMObject to consider the relation for.
    """

    # specifies the maximum distance in metres between objects for relations to exist.
    dist_thr = 3.0

    relation = Relation()
    robot_pos = np.array([cur_robot_pose.x, cur_robot_pose.y, cur_robot_pose.z])
    obj_one_pos = np.array([som_obj_one.pose_estimate.x, som_obj_one.pose_estimate.y, som_obj_one.pose_estimate.z])
    obj_two_pos = np.array([som_obj_two.pose_estimate.x, som_obj_two.pose_estimate.y, som_obj_two.pose_estimate.z])

    robot_to_two = obj_two_pos - robot_pos
    two_to_one = obj_one_pos - obj_two_pos

    # if the distance between the two objects is greater than the threshold then none of the relations are true
    if np.linalg.norm(obj_one_pos - obj_two_pos) > dist_thr:
        return relation
    else:
        relation.at = True

    # vertical relation
    if som_obj_one.pose.z > som_obj_two.pose.z:
        relation.above = True
        relation.ontop = True
    else:
        relation.below = True

    # forwards and backwards
    if np.dot(robot_to_two, two_to_one) > 0.0:
        relation.behind = True
    else:
        relation.frontof = True

    # left and right
    cross_pr = np.cross(robot_to_two, two_to_one)
    if cross_pr[2] > 0.0:
        relation.left = True
    else:
        relation.right = True

    return relation