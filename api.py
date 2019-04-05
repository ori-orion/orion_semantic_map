"""
Defines the barebones API for SOM.
"""



class SomObject(object):
    """A class that templates information that we might want to store about an 
    object
    
    Attributes (all optional):
        _id: A unique id for this object, that 
        _ontology_concept: A concept from our ontology. This concept should be 
                           the most specific possible (i.e. if we can detect 
                           that something is a 'bottle of coke', we shouldn't be 
                           storing the concept as 'drink'.)
        _ontology_properties: A dictionary mapping from string names to values, 
                              representing values for the properties associated 
                              with the _ontology_concept
        _location_observation: A <TODO> object, representing the objects 
                              location (and orientation)
        _location_estimate: A probability distribution over <TODO> objects, 
                           representing where we think this SomObject may be.
        _size: The size of the object
        _weight: The weight of the object
        _task_roll: The roll of the object in a given task
        _name: String for the name of a person
        _age: An age for a person
        _pose: A pose for a person
        _gender: The gender of a person
        _shirt_colour: The shirt colour that a person is wearing
        _meta_properties: A list of arbitrary objects, for additional 
                          properties that are specific to cirtain types of 
                          SomObject instance
    TODO:
        * Decide on the correct type for a location (a tf?)
    """
    def __init__(self):
        pass
    def get_id(self):
        pass
    def set_id(self):
        pass
    def get_ontology_concept(self):
        pass
    def set_ontology_concept(self):
        pass
    def get_ontology_properties(self):
        pass
    def set_ontology_properties(self):
        pass
    def get_location_observation(self):
        pass
    def set_location_observation(self):
        pass
    def get_location_estimate(self):
        pass
    def set_location_estimate(self):
        pass # potentially delete this if want to make it read only
    def get_size(self):
        pass
    def set_size(self):
        pass
    def get_weight(self):
        pass
    def set_weight(self):
        pass
    def get_task_roll(self):
        pass
    def set_task_roll(self):
        pass
        
    def get_name(self):
        pass
    def set_name(self):
        pass
    def get_age(self):
        pass
    def set_age(self):
        pass
    def get_pose(self):
        pass
    def set_pose(self):
        pass
    def get_gender(self):
        pass
    def set_gender(self):
        pass
    def get_shirt_colour(self):
        pass
    def set_shirt_colour(self):
        pass

    def get_meta_properties(self):
        pass
    def set_meta_properties(self):
        pass



def store_observation(object, time):
    """
    Registers an observation with the SOMa backend. 'object' should contain as 
    much information as we can provide about the observation, and time is a 
    timestamp for which it was registered.

    Internals:
    1. The observation itself should be stored, to allow for 'rosbag' style 
       replay of observations.
    2. We may wish to do some pre-processing here, but in general, filling in 
       missing values should be performed at query time.

    Args:
        object: An instance of SomObject that was observed
        time: The timestamp for which the observation was made
    Returns:
        Nothing
    """
    pass



def query(x=None, tilde=None, y=None):
    """
    Queries for the relation x~y, where 'x' and 'y' are instances of SomObject 
    and '~' is an instance of Relation. Of 'x', '~' and 'y', only one or two 
    should be specified.

    This function will then return a list (of pairs if necessary) that complete 
    valid (x,~,y) tuples. This is best demonstrated with example I/O given 
    below:
        query(x=milk_instance, tilde=Relation.BELOW, y=None)
            -> [coke_soda_instance, fanta_soda_instance, cheese_instance]
        query(x=None, tilde=Relation.ONTOP, y=None)
            -> [(table_instance, coke_soda_instance), 
                (table_instance, food_plate_instance),
                (shelf_instance, water_bottle_instance)]
        query(x=michael_instance, tilde=None, y=table_instance)
            -> [Relation.AT]
        query(x=table_instance, tilde=None, y=michael_instance)
            -> []

    Args:
        x: A SomObject instance that we wish to make a query about
        tilde: A Relation instance that we wish to make a query about
        y: A SomObject instance that we wish to make a query about
    Returns:
        A list of completions for valid (x,~,y) relation tuples for the given 
        input.
    Raises:
        ValueError: If invalid input is given to the function
    TODO:
        * Implement query(None, None, y) as query(y, None, None) and so on, to 
          simplify code.
        * Make sure to check that they havent provided 0 or 3 parameters?
    """
    pass