"""
Defines the barebones API for SOMObjects.
"""



from semantic_mapping.msg import SOMObservation



class SOMObject(object):
    """A class that templates information that we might want to store about an 
    object
    
    Attributes (all optional):
        _id: A unique id for this object, that 
        _map_name: <TODO>
        _map_unique_id: <TODO>
        _type: <TODO>
        # _ontology_concept: A concept from our ontology. This concept should be 
        #                    the most specific possible (i.e. if we can detect 
        #                    that something is a 'bottle of coke', we shouldn't be 
        #                    storing the concept as 'drink'.)
        # _ontology_properties: A dictionary mapping from string names to values, 
        #                       representing values for the properties associated 
        #                       with the _ontology_concept
        _location_observations: A <TODO> object, representing the objects 
                                list of poses the object has been observed in
        _location_estimate: A probability distribution over <TODO> objects, 
                            representing where we think this SomObject may be.
        _size: The size of the object
        _weight: The weight of the object
        _task_role: The role of the object in a given task
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
    def get_map_name(self):
        pass
    def set_map_name(self):
        pass
    # def get_ontology_concept(self):
    #     pass
    # def set_ontology_concept(self):
    #     pass
    # def get_ontology_properties(self):
    #     pass
    # def set_ontology_properties(self):
    #     pass
    def get_pose_observations(self):
        pass
    def set_pose_observations(self):
        pass
    def get_size(self):
        pass
    def set_size(self):
        pass
    def get_weight(self):
        pass
    def set_weight(self):
        pass
    def get_task_role(self):
        pass
    def set_task_role(self):
        pass
        
    def get_name(self):
        pass
    def set_name(self):
        pass
    def get_age(self):
        pass
    def set_age(self):
        pass
    def get_posture(self):
        pass
    def set_posture(self):
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



    def get_room_distribution(self):
    	"""
		TODO: return a distributuion over waypoints/rooms where we 
		think this object may actually be.
    	"""
    	pass


    def get_pose_estimate(self):
    	"""
		TODO: return a distributuion over waypoints/rooms where exactly 
		we think this object may actually be.
    	"""
        pass

    @classmethod
    def from_som_observation_message(self, som_observation_msg):
    	obj = SomObject()
    	
    	obj.set_id(som_observation_msg.id)
    	obj.set_map_name(som_observation_msg.map_name)
    	obj.set_meta_properties(som_observation_msg.meta_properties)
    	obj.set_type(som_observation_msg.type)
    	obj.set_timestamp(som_observation_msg.timestamp)
    	obj.set_size(som_observation_msg.size)
    	obj.set_weight(som_observation_msg.weight)
    	obj.set_task_role(som_observation_msg.task_role)
    	obj.set_pose_observation(som_observation_msg.pose_observation)
    	obj.set_robot_pose(som_observation_msg.robot_pose)
    	obj.set_cloud(som_observation_msg.cloud)
    	obj.set_colour(som_observation_msg.colour)
    	obj.set_name(som_observation_msg.name)
    	obj.set_age(som_observation_msg.age)
    	obj.set_posture(som_observation_msg.posture)
    	obj.set_gender(som_observation_msg.gender)
    	obj.set_shirt_colour(som_observation_msg.shirt_colour)

    	return obj


    def to_som_oservation_message(self):
    	obj = SOMObservationMsg()

    	if self._id is not None:
    		obj.id = self._id
    	if self._map_name is not None:
    		obj.map_name = self._map_name
    	if self._meta_properties is not None:
    		obj.meta_properties = str(self._meta_properties)
    	if self._type is not None:
    		obj.type = self._type
    	if self._timestamp is not None:
    		obj.timestamp = self._timestamp
    	if self._size is not None:
    		obj.size = self._size
    	if self._weight is not None:
    		obj.weight = self._weight
    	if self._task_role is not None:
    		obj.task_role = self._task_role
    	if self._pose_observation is not None:
    		obj.pose_observation = self._pose_observation
    	if self._robot_pose is not None:
    		obj.robot_pose = self._robot_pose
    	if self._cloud is not None:
    		obj.cloud = self._cloud
    	if self._colour is not None:
    		obj.colour = self._colour
    	if self._name is not None:
    		obj.name = self._name
    	if self._age is not None:
    		obj.age = self._age
    	if self._posture is not None:
    		obj.posture = self._posture
    	if self._gender is not None:
    		obj.gender = self._gender
    	if self._shirt_colour is not None:
    		obj.shirt_colour = self._shirt_colour

    	return obj
