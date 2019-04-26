"""
Defines the barebones API for InSOMObjects.
"""



from semantic_mapping.msg import SOMObservation




class InSOMObject(object):
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
                            representing where we think this InSomObject may be.
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
                          InSomObject instance
        _room_geometry: ...
        _
    TODO:
        * Decide on the correct type for a location (a tf?)
    """
    def __init__(self):
        pass

    def get_id(self):
        return self._id

    def set_id(self, id):
        self._id = id

    def get_map_name(self):
        return self._map_name

    def set_map_name(self, map_name):
        self._map_name

    # def get_ontology_concept(self):
    #     pass
    # def set_ontology_concept(self):
    #     pass
    # def get_ontology_properties(self):
    #     pass
    # def set_ontology_properties(self):
    #     pass
    # def get_pose_observations(self):
    #     pass
    # def set_pose_observations(self):
    #     pass

    def get_pose_estimate(self):
        return self._pose_estimate

    def set_pose_estimate(self, pose_estimate):
        self._pose_estimate = pose_estimate

    def get_size(self):
        return self._size

    def set_size(self, size):
        self._size = size

    def get_weight(self):
        return self._weight

    def set_weight(self, weight):
        self._weight = weight

    def get_task_role(self):
        return self._task_role 

    def set_task_role(self, task_role):
        self._task_role = task_role 
        
    def get_name(self):
        return self._name

    def set_name(self, name):
        self._name = name 

    def get_age(self, age):
        return self._age

    def set_age(self, age):
        self._age = age

    def get_posture(self, posture):
        return self._posture

    def set_posture(self, posture):
        self._posture = posture

    def get_gender(self):
        return self._gender

    def set_gender(self, gender):
        self._gender = gender 

    def get_shirt_colour(self):
        return self._shirt_colour

    def set_shirt_colour(self, shirt_colour):
        self._shirt_colour = shirt_colour

    def get_cloud(self):
        return self._cloud

    def set_cloud(self, cloud):
        self._cloud = cloud

    def get_room_geometry(self):
        return self._room_geometry

    def set_room_geometry(self, room_geometry):
        self._room_geometry = room_geometry

    def get_room_name(self):
        return self._room_name

    def set_room_name(self, room_name):
        self._room_name = room_name

    def get_waypoint(self):
        return self._waypoint

    def set_waypoint(self, waypoint):
        self._waypoint = waypoint 

    def get_meta_properties(self):
        return self._meta_properties

    def set_meta_properties(self, meta_properties):
        self._meta_properties = meta_properties

    def get_room_distribution(self):
    	"""
		TODO: return a distributuion over waypoints/rooms where we 
		think this object may actually be.
    	"""
    	pass


    @classmethod
    def from_som_observation_message(self, som_observation):
        obj = InSomObject()
        
        obj.set_id(som_observation.id)
        obj.set_map_name(som_observation.map_name)
        obj.set_meta_properties(som_observation.meta_properties)
        obj.set_type(som_observation.type)
        obj.set_timestamp(som_observation.timestamp)
        obj.set_size(som_observation.size)
        obj.set_weight(som_observation.weight)
        obj.set_task_role(som_observation.task_role)
        # obj.set_pose_estimate(som_observation.pose_estimate)
        # obj.set_robot_pose(som_observation.robot_pose)
        obj.set_cloud(som_observation.cloud)
        obj.set_room_name(som_observation_mgs.room_name)
        obj.set_waypoint(som_observation_mgs.waypoint)
        obj.set_room_geometry(som_observation.room_geometry)
        obj.set_colour(som_observation.colour)
        obj.set_name(som_observation.name)
        obj.set_age(som_observation.age)
        obj.set_posture(som_observation.posture)
        obj.set_gender(som_observation.gender)
        obj.set_shirt_colour(som_observation.shirt_colour)

        return obj
        

    def to_som_observation_message(self):
        obj = SOMObservation()

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
        # if self._pose_estimate is not None:
        #     obj.pose_observation = self._pose_estimate
        # if self._robot_pose is not None:
        #     obj.robot_pose = self._robot_pose
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


    @classmethod
    def from_som_object_message(self, som_observation):
        obj = InSomObject()
        
        obj.set_id(som_observation.id)
        obj.set_map_name(som_observation.map_name)
        obj.set_meta_properties(som_observation.meta_properties)
        obj.set_type(som_observation.type)
        obj.set_timestamp(som_observation.timestamp)
        obj.set_size(som_observation.size)
        obj.set_weight(som_observation.weight)
        obj.set_task_role(som_observation.task_role)
        obj.set_pose_estimate(som_observation.pose_estimate)
        # obj.set_robot_pose(som_observation.robot_pose)
        obj.set_cloud(som_observation.cloud)
        obj.set_room_name(som_observation_mgs.room_name)
        obj.set_waypoint(som_observation_mgs.waypoint)
        obj.set_room_geometry(som_observation.room_geometry)
        obj.set_colour(som_observation.colour)
        obj.set_name(som_observation.name)
        obj.set_age(som_observation.age)
        obj.set_posture(som_observation.posture)
        obj.set_gender(som_observation.gender)
        obj.set_shirt_colour(som_observation.shirt_colour)

        return obj
        

    def to_som_object_message(self):
        obj = SOMObject()

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
        if self._pose_estimate is not None:
            obj.pose_estimate = self._pose_estimate
        # if self._robot_pose is not None:
        #     obj.robot_pose = self._robot_pose
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


    def dict_iter(self):
        """
        Iterates through the non-null values, and returns a (string, value) 
        pair for each one. THe string is the 
        """
        if self._id is not None:
            yield "obj_id", self._id
        if self._map_name is not None:
            yield "map_name", self._map_name
        # if self._meta_properties is not None:
        #     yield "meta_properties", self._meta_properties
        if self._type is not None:
            yield "type", self._type
        if self._timestamp is not None:
            yield "timestamp", self._timestamp
        if self._size is not None:
            yield "size", self._size
        if self._weight is not None:
            yield "weight", self._weight
        if self._task_role is not None:
            yield "task_role", self._task_role
        if self._pose_estimate is not None:
            yield "pose_estimate", self._pose_estimate
        # if self._size is not None:
        #     yield "size", self._size
        # if self._robot_pose is not None:
        #     yield "robot_pose", self._robot_pose
        # if self._cloud is not None:
        #     yield "cloud", self._cloud
        if self._colour is not None:
            yield "colour", self._colour
        if self._room_name is not None:
            yield "room_name", self._room_name
        if self._waypoint is not None:
            yield "waypoint", self._waypoint
        # if self._room_geometry is not None:
        #     yield "room_geometry", self._room_geometry
        if self._name is not None:
            yield "name", self._name
        if self._age is not None:
            yield "age", self._age
        if self._posture is not None:
            yield "posture", self._posture
        if self._gender is not None:
            yield "gender", self._gender
        if self._shirt_colour is not None:
            yield "shirt_colour", self._shirt_colour


    def to_som_object_mongo_db_query(self):
        """
        Returns a dictionary 'd' to be used as follows:
        msg_store.query(SOMObservation._type, d)

        The objects that match this dictionary in the data store should be 
        InSOMObject objects.

        An example of a mongo db query is:
        msg_store.query(Pose._type, {"position.y": 1})

        http://wiki.ros.org/mongodb_store
        https://www.w3schools.com/python/python_mongodb_query.asp
        """
        # First check the things that we don't know how to handle
        if self._meta_properties is not None or
           self._size is not None or 
           self._pose_estimate is not None or
           self._robot_pose is not None or
           self._cloud is not None or
           self._room_geometry is not None:
            raise Exception("Your query is currently not supported. Come bug "
                            "the semantic mapping team to fix this.")

        return {key: value for key, value in self.dict_iter()}


    def from_som_object_message(self):
        pass