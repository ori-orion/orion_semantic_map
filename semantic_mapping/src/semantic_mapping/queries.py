# -*- coding: utf-8 -*-

import numpy as np
from som_object import InSOMObject
from orion_actions.msg import Match, Relation, SOMObject

def query(som_template_one, relation, som_template_two, cur_robot_pose, mongo_object_store, ontology):
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
    som_obj1 = InSOMObject.from_som_observation_message(som_template_one)
    som_obj2 = InSOMObject.from_som_observation_message(som_template_two)
    query_dict1 = som_obj1.to_som_object_mongo_db_query()
    query_dict2 = som_obj2.to_som_object_mongo_db_query()

    # if only a single object is specified
    if (len(query_dict1) == 0 or len(query_dict2) == 0) and unspecified_relation(relation):
        matches = _match_single_object(query_dict1, query_dict2, mongo_object_store, ontology)
    else:
        matches = _match_with_relation(query_dict1, relation, query_dict2, mongo_object_store, ontology, cur_robot_pose)
    return matches

def _match_single_object(query_dict1, query_dict2, mongo_object_store, ontology):
    '''
    Generates matching tuples when only a single query and no relation is specified.

    This is necessary because _match_with_relation does not handle this case properly.
    For example, if the database were to contain orange1 and orange2, and we
    attempt to query for oranges in the database:
    query(orange, -, -)
    _match_single_object returns:
    Match(orange1,-,-)
    Match(orange2,-,-)
    as desired. _match_with_relation would instead return:
    Match(orange1,rel,orange1)
    Match(orange1,rel,orange2)
    Match(orange2,rel,orange1)
    Match(orange2,rel,orange2)
    which is not the appropriate list of matches for this case.
    '''
    matches = []
    if len(query_dict2) == 0:
        objects = _mongo_som_objects_matching_template(query_dict1, mongo_object_store, ontology)
        for object in objects:
            match = Match(object, Relation(), SOMObject())
            matches.append(match)
    else:
        objects = _mongo_som_objects_matching_template(query_dict2, mongo_object_store, ontology)
        for object in objects:
            match = Match(SOMObject(), Relation(), object)
            matches.append(match)
    return matches

def _match_with_relation(query_dict1, relation, query_dict2, mongo_object_store, ontology, cur_robot_pose):
    '''
    Generates matching tuples when more than a single object template is specified.
    '''
    o1_matches = _mongo_som_objects_matching_template(query_dict1, mongo_object_store, ontology)
    o2_matches = _mongo_som_objects_matching_template(query_dict2, mongo_object_store, ontology)

    matches = []
    for o1 in o1_matches:
        for o2 in o2_matches:
            rel = spatial_relation(cur_robot_pose, o1, o2)
            matching = True
            strings = relation.__str__().split('\n')

            for str in strings:
                key = str.split(':')[0]
                if relation.__getattribute__(key) and not rel.__getattribute__(key):
                    matching = False

            if matching:
                match = Match(o1, rel, o2)
                matches.append(match)
    return matches

def _mongo_som_objects_matching_template(query_dict, mongo_object_store, ontology):
    """
    This returns a list of SOMObjects in the database matching the query dictionary.

    Args:
        query: query dictionary
        mongo_object_store: A mongodb store for SomObject instances
        ontology: Instance of Ontology class
    Returns:
        A list of objects matching the template.
    """

    # If query includes type return matching types including children in ontology
    if 'type' in query_dict:
        if not ontology.check_class_exists(query_dict['type']):
            raise Exception('Type specified in observation is not valid ontology class')
            return SOMObserveResponse(False, '')
        valid_types = ontology.get_valid_types(query_dict['type'])

        results = []
        for valid_type in valid_types:
            query_dict['type'] = valid_type.lower()
            response = mongo_object_store.query(SOMObject._type, message_query=query_dict)
            result = [i[0] for i in response]
            results = results + result
    else:
        response = mongo_object_store.query(SOMObject._type, message_query=query_dict)
        results = [i[0] for i in response]
    return results

def spatial_relation(cur_robot_pose, som_obj_one, som_obj_two):
    """
    This should return the relation ~ that makes the statement
    "som_obj_one ~ som_obj_two" true.
uples
    We assume that only one spatial relation can be true at a time?

    This can likely be computed using dot products, and checking which
    side of a given plane we are. So we say define 6/8 planes, and
    depending on the signs of dot products, we give a spatial relation.

    One complexity is how to consider the uplesshape of the object. For
    example, the pose of a table may be the central point of the table.
    So if an object is on the left of the table, it's not clear how to
    return that the object is "ontop" the table, rather than to the
    "left" of. I guess object shapes should be taken into account? Or
    we try to make sure that the poses are at the center of mass?

    Args:
        cur_robot_pose: The current pose of the robot, used as a
                        reference frame to work out spatial relations.
        som_obj_one: The first InSOMObject to consider the relation for.
        som_obj_two: The second InSOMObjecuplest to consider the relation for.
    """

    # specifies the maximum distance in metres between objects for relations to exist.
    dist_thr = 3.0

    relation = Relation()
    robot_pos = np.array([cur_robot_pose.position.x, cur_robot_pose.position.y, cur_robot_pose.position.z])
    obj_one_pos = np.array([som_obj_one.pose_estimate.most_likely_pose.position.x, som_obj_one.pose_estimate.most_likely_pose.position.y, som_obj_one.pose_estimate.most_likely_pose.position.z])
    obj_two_pos = np.array([som_obj_two.pose_estimate.most_likely_pose.position.x, som_obj_two.pose_estimate.most_likely_pose.position.y, som_obj_two.pose_estimate.most_likely_pose.position.z])

    robot_to_two = obj_two_pos - robot_pos
    two_to_one = obj_one_pos - obj_two_pos

    # if the distance between the two objects is greater than the threshold then none of the relations are true
    if np.linalg.norm(obj_one_pos - obj_two_pos) > dist_thr:
        return relation
    else:
        relation.near = True

    # vertical relation
    eps = 0.001 # use tolerance otherwise comparing relations of object to itself returns true for some fields
    if obj_one_pos[2] > obj_two_pos[2] + eps:
        relation.above = True
    elif obj_one_pos[2] < obj_two_pos[2] - eps:
        relation.below = True

    # forwards and backwards
    if np.dot(robot_to_two, two_to_one) > eps:
        relation.behind = True
    elif np.dot(robot_to_two, two_to_one) < -eps:
        relation.frontof = True

    # left and right
    cross_pr = np.cross(robot_to_two, two_to_one)
    if cross_pr[2] > eps:
        relation.left = True
    elif cross_pr[2] < -eps:
        relation.right = True

    return relation

def unspecified_relation(relation):
    '''
    Checks if none of the fields of a relation are specified to be true.

    Args:
        relation: an instance of the Relation class
    Returns:
        A boolean indicating whether all of the fields are false
    '''
    all_false = True
    strings = relation.__str__().split('\n')
    for str in strings:
        key = str.split(':')[0]
        if relation.__getattribute__(key):
            all_false = False
    return all_false