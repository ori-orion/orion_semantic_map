"""
Defines the barebones API for InSOMObjects.
"""



from orion_actions.msg import SOMObservation, SOMObject, PoseEstimate
import shapely.geometry as geom



def _default_value(x):
    return (x is None or x == 0 or x == "" or x == PoseEstimate())




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
        self._obj_id = None
        self._map_name = None
        self._meta_properties = None
        self._type = None
        self._timestamp = None
        self._size = None
        self._weight = None
        self._task_role = None
        self._cloud = None
        self._colour = None
        self._name = None
        self._age = None
        self._posture = None
        self._gender = None
        self._shirt_colour = None
        self._coat_colour = None
        self._drink = None
        self._observed = None

    def get_id(self):
        return self._obj_id

    def set_id(self, id):
        self._obj_id = id

    def get_map_name(self):
        return self._map_name

    def set_map_name(self, map_name):
        self._map_name = map_name

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

    def get_timestamp(self):
        return self._pose_estimate

    def set_timestamp(self, timestamp):
        self._timestamp = timestamp

    def get_size(self):
        return self._size

    def set_size(self, size):
        self._size = size

    def get_weight(self):
        return self._weight

    def set_weight(self, weight):
        self._weight = weight

    def get_colour(self):
        return self._colour

    def set_colour(self, colour):
        self._colour = colour

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

    def get_type(self):
        return self._type

    def set_type(self, type):
        self._type = type

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

    def get_coat_colour(self):
        return self._coat_colour

    def set_coat_colour(self, coat_colour):
        self._coat_colour = coat_colour

    def get_drink(self):
        return self._drink

    def set_drink(self, drink):
        self._drink = drink

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

    def set_observed(self, observed):
        self._observed = observed

    def get_room_distribution(self):
    	"""
		TODO: return a distributuion over waypoints/rooms where we
		think this object may actually be.
    	"""
    	pass


    @classmethod
    def from_som_observation_message(self, som_observation):
        obj = InSOMObject()

        obj.set_id(som_observation.obj_id)
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
        obj.set_room_name(som_observation.room_name)
        obj.set_waypoint(som_observation.waypoint)
        obj.set_room_geometry(som_observation.room_geometry)
        obj.set_colour(som_observation.colour)
        obj.set_name(som_observation.name)
        obj.set_age(som_observation.age)
        obj.set_posture(som_observation.posture)
        obj.set_gender(som_observation.gender)
        obj.set_shirt_colour(som_observation.shirt_colour)
        obj.set_coat_colour(som_observation.coat_colour)
        obj.set_drink(som_observation.drink)
        obj.set_observed(True)

        return obj

    @classmethod
    def from_som_object_message(self, som_object):
        obj = InSOMObject()

        obj.set_id(som_object.obj_id)
        obj.set_map_name(som_object.map_name)
        obj.set_meta_properties(som_object.meta_properties)
        obj.set_type(som_object.type)
        obj.set_timestamp(som_object.timestamp)
        obj.set_size(som_object.size)
        obj.set_weight(som_object.weight)
        obj.set_task_role(som_object.task_role)
        obj.set_pose_estimate(som_object.pose_estimate)
        # obj.set_robot_pose(som_object.robot_pose)
        obj.set_cloud(som_object.cloud)
        obj.set_room_name(som_object.room_name)
        obj.set_waypoint(som_object.waypoint)
        obj.set_room_geometry(som_object.room_geometry)
        obj.set_colour(som_object.colour)
        obj.set_name(som_object.name)
        obj.set_age(som_object.age)
        obj.set_posture(som_object.posture)
        obj.set_gender(som_object.gender)
        obj.set_shirt_colour(som_object.shirt_colour)
        obj.set_coat_colour(som_object.coat_colour)
        obj.set_drink(som_object.drink)
        obj.set_observed(True)
        
        return obj

    def to_som_observation_message(self):
        obj = SOMObservation()

        if not _default_value(self._obj_id):
            obj.obj_id = self._obj_id
        if not _default_value(self._map_name):
            obj.map_name = self._map_name
        if not _default_value(self._meta_properties):
            obj.meta_properties = str(self._meta_properties)
        if not _default_value(self._type):
            obj.type = self._type
        if not _default_value(self._timestamp):
            obj.timestamp = self._timestamp
        if not _default_value(self._size):
            obj.size = self._size
        if not _default_value(self._weight):
            obj.weight = self._weight
        if not _default_value(self._task_role):
            obj.task_role = self._task_role
        # if not _default_value(self._pose_estimate):
        #    obj.pose_estimate = self._pose_estimate
        # if not _default_value(self._robot_pose):
        #     obj.robot_pose = self._robot_pose
        if not _default_value(self._cloud):
            obj.cloud = self._cloud
        if not _default_value(self._colour):
            obj.colour = self._colour
        if not _default_value(self._name):
            obj.name = self._name
        if not _default_value(self._age):
            obj.age = self._age
        if not _default_value(self._posture):
            obj.posture = self._posture
        if not _default_value(self._gender):
            obj.gender = self._gender
        if not _default_value(self._shirt_colour):
            obj.shirt_colour = self._shirt_colour
        if not _default_value(self._coat_colour):
            obj.coat_colour = self._coat_colour
        if not _default_value(self._drink):
            obj.drink = self._drink

        return obj

    def to_som_object_message(self):
        obj = SOMObject()

        if not _default_value(self._obj_id):
            obj.obj_id = self._obj_id
        if not _default_value(self._map_name):
            obj.map_name = self._map_name
        if not _default_value(self._meta_properties):
            obj.meta_properties = str(self._meta_properties)
        if not _default_value(self._type):
            obj.type = self._type
        if not _default_value(self._timestamp):
            obj.timestamp = self._timestamp
        if not _default_value(self._size):
            obj.size = self._size
        if not _default_value(self._weight):
            obj.weight = self._weight
        if not _default_value(self._task_role):
            obj.task_role = self._task_role
        if not _default_value(self._pose_estimate):
            obj.pose_estimate = self._pose_estimate
        # if not _default_value(self._robot_pose):
        #     obj.robot_pose = self._robot_pose
        if not _default_value(self._cloud):
            obj.cloud = self._cloud
        if not _default_value(self._colour):
            obj.colour = self._colour
        if not _default_value(self._room_name):
            obj.room_name = self._room_name
        if not _default_value(self._name):
            obj.name = self._name
        if not _default_value(self._age):
            obj.age = self._age
        if not _default_value(self._posture):
            obj.posture = self._posture
        if not _default_value(self._gender):
            obj.gender = self._gender
        if not _default_value(self._shirt_colour):
            obj.shirt_colour = self._shirt_colour
        if not _default_value(self._drink):
            obj.drink = self._drink
        if not _default_value(self._coat_colour):
            obj.coat_colour = self._coat_colour
        obj.observed = self._observed

        return obj

    def update_from_observation_messages(self, observations, rois):
        '''
        Given a new observation message updates a SOM object.

        The properties automatically updated are the pose estimate and room name.
        Any other properties which have been specified are also updated.
        '''
        pose_estimate = self.estimate_pose(observations, rois)
        most_recent_observation = observations[-1]
        self.update_properties_from_obs_msg(most_recent_observation)
        self.set_pose_estimate(pose_estimate)
        self.set_room_name(self.get_room_name(pose_estimate, rois))

    def update_properties_from_obs_msg(self, som_observation):
        ''' For an existing InSOMObject this method uses a new observation to
        update properties if the properties are not default values in the new
        observation
        '''
        if not _default_value(som_observation.obj_id):
            self._obj_id = som_observation.obj_id
        if not _default_value(som_observation.map_name):
            self._map_name = som_observation.map_name
        if not _default_value(som_observation.meta_properties):
            self._meta_properties = str(som_observation.meta_properties)
        if not _default_value(som_observation.type):
            self._type = som_observation.type
        if not _default_value(som_observation.timestamp):
            self._timestamp = som_observation.timestamp
        if not _default_value(som_observation.size):
            self._size = som_observation.size
        if not _default_value(som_observation.weight):
            self._weight = som_observation.weight
        if not _default_value(som_observation.task_role):
            self._task_role = som_observation.task_role
        # if not _default_value(self._pose_estimate):
        #    obj.pose_estimate = self._pose_estimate
        # if not _default_value(self._robot_pose):
        #     obj.robot_pose = self._robot_pose
        if not _default_value(som_observation.cloud):
            self._cloud = som_observation.cloud
        if not _default_value(som_observation.colour):
            self._colour = som_observation.colour
        if not _default_value(som_observation.name):
            self._name = som_observation.name
        if not _default_value(som_observation.age):
            self._age = som_observation.age
        if not _default_value(som_observation.posture):
            self._posture = som_observation.posture
        if not _default_value(som_observation.gender):
            self._gender = som_observation.gender
        if not _default_value(som_observation.shirt_colour):
            self._shirt_colour = som_observation.shirt_colour
        if not _default_value(som_observation.coat_colour):
            self._coat_colour = som_observation.coat_colour
        if not _default_value(som_observation.drink):
            self._drink = som_observation.drink

    def dict_iter(self):
        """
        Iterates through the non-null values, and returns a (string, value)
        pair for each one. THe string is the
        """
        if not _default_value(self._obj_id):
            yield "obj_id", self._obj_id
        if not _default_value(self._map_name):
            yield "map_name", self._map_name
        # if not _default_value(self._meta_properties):
        #     yield "meta_properties", self._meta_properties
        if not _default_value(self._type):
            yield "type", self._type
        if not _default_value(self._timestamp):
            yield "timestamp", self._timestamp
        # if not _default_value(self._size):
        #    yield "size", self._size
        if not _default_value(self._weight):
            yield "weight", self._weight
        if not _default_value(self._task_role):
            yield "task_role", self._task_role
        # if not _default_value(self._pose_estimate):
        #    yield "pose_estimate", self._pose_estimate
        # if not _default_value(self._size):
        #     yield "size", self._size
        # if not _default_value(self._robot_pose):
        #     yield "robot_pose", self._robot_pose
        # if not _default_value(self._cloud):
        #     yield "cloud", self._cloud
        if not _default_value(self._colour):
            yield "colour", self._colour
        if not _default_value(self._room_name):
            yield "room_name", self._room_name
        if not _default_value(self._waypoint):
            yield "waypoint", self._waypoint
        # if not _default_value(self._room_geometry):
        #     yield "room_geometry", self._room_geometry
        if not _default_value(self._name):
            yield "name", self._name
        if not _default_value(self._age):
            yield "age", self._age
        if not _default_value(self._posture):
            yield "posture", self._posture
        if not _default_value(self._gender):
            yield "gender", self._gender
        if not _default_value(self._shirt_colour):
            yield "shirt_colour", self._shirt_colour
        if not _default_value(self._coat_colour):
            yield "coat_colour", self._coat_colour
        if not _default_value(self._drink):
            yield "drink", self._drink


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
        # TODO: below is giving me a syntax error so needs to be fixed
        #if self._meta_properties is not None or
        #   self._size is not None or
        #   self._pose_estimate is not None or
        #   self._robot_pose is not None or
        #   self._cloud is not None or
        #   self._room_geometry is not None:
        #    raise Exception("Your query is currently not supported. Come bug "
        #                    "the semantic mapping team to fix this.")

        return {key: value for key, value in self.dict_iter()}

    def get_room_name(self, pose_estimate, rois):
        '''Given a PoseEstimate and a list of SomROIObjects, returns the room
        name that the PoseEstimate is in, based on the most_likely_pose. If the
        PoseEstimate is not in a room, returns "NotInRoom"
        '''
        obj_pose = pose_estimate.most_likely_pose
        obj_point = geom.Point(obj_pose.position.x, obj_pose.position.y)


        for roi in rois:
            vertex_poses = roi.posearray.poses
            vertex_tuples = []
            for vertex_pose in vertex_poses:
                vertex_tuples.append((vertex_pose.position.x, vertex_pose.position.y))
            roi_poly = geom.polygon.Polygon(vertex_tuples)

            if roi_poly.contains(obj_point):
                return roi.name
        return "NotInRoom"

    def estimate_pose(self, observations, rois):
        ''' Receives a list of observations and returns pose_estimate object.

        TODO: improve this.
        '''
        pose_estimate = PoseEstimate()
        pose_estimate.most_likely_pose = observations[-1].pose_observation
        pose_estimate.most_recent_pose = observations[-1].pose_observation
        pose_estimate.most_likely_room = self.get_room_name(pose_estimate, rois)

        room_names = []
        room_probs = []
        for roi in rois:
            room_names.append(roi.name)
            if roi.name == pose_estimate.most_likely_room:
                room_probs.append(0.8)
            else:
                room_probs.append(0.2/(len(rois)-1))

        pose_estimate.room_probs = room_probs
        pose_estimate.room_names = room_names
        return pose_estimate
