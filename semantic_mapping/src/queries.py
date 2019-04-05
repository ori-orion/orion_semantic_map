from .constants import Relation


def _get_all_relations():
    """Returns a list of all possible Relation enum values."""
    return [Relation.LEFT, 
    		Relation.RIGHT,
    		Relation.ABOVE, 
    		Relation.BELOW,
    		Relation.AT,
    		Relation.ONTOP]


def query(som_template_one, relation, som_template_two, cur_robot_pose):
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
        som_obj_one: A SOMObject to be used as a template to look up 
            objects in the mongodb. 
        relation: A Relation enum type specifying the relation 
            required between objects, or, None, if we want to consider 
            all possible relations.
        som_obj_two: A SOMObject to be used as a template to look up 
            objects in the mongodb.
        cur_robot_pos: The pose of the current robot, used to compute 
            spatial relations. (These change depending on where the 
            robot is facing).

    Returns:
        All valid (o1, ~, o2) tuples, for which o1 ~ o2 is true and 
        o1 matches the template of 'som_obj_one' and o2 matches the 
        template of 'som_obj_two'.
    """
    o1_matches = _mongo_som_objects_matching_template(som_template_one)
    o2_matches = _mongo_som_objects_matching_template(som_template_two)
    relations = [relation] if relation is not None else _get_all_relations()

    tuples = []
    for o1 in o1_matches:
        for o2 in o2_matches:
            rel = spatial_relation(cur_robot_pose, o1, o2)
            if rel in relations:
                matching_tuple = (o1, rel, o2)
                tuples.append(matching_tuple)

    return tuples

def _mongo_som_objects_matching_template(som_obj):
    """
    Treats the SOMObject 'som_ojb' as a template to match in mongo db. 
    This returns a list of SOMObjects in the database matching the 
    template. 

    Args:
        som_obj: The SOMObject (template) specified to make a query
    Returns:
        A list of objects matching the template.
    """
    som_observation_msg = som_obj.to_som_observation_message()
    # todo: make a dictionary from observation method (use an iterator? over the observation message? in SOMObject class)
    # todo: query to get a list of observations (?objects?) from the mongodb
    # todo: convert to som objects
    # todo: return list
    # use: http://wiki.ros.org/mongodb_store
    # use: https://www.w3schools.com/python/python_mongodb_query.asp

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
        som_obj_one: The first SOMObject to consider the relation for.
        som_obj_two: The second SOMObject to consider the relation for.
    """
    pass