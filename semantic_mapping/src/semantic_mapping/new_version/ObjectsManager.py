import rospy

from geometry_msgs.msg import Point

import orion_actions.srv as act_srv
import orion_actions.msg as act_msg


from typing import Any, Dict, List, Optional, Union
from enum import Enum
import numpy as np
from bson import ObjectId
import pymongo
import pymongo.cursor
import pymongo.collection

import new_version.profiler as prof

from new_version.constants import (
    DEBUG_LONG,
    HUMAN_CLASS,
    NON_PICKUPABLE_OBJS,
    PYMONGO_ID_KEY,
    SERVICE_ROOT,
    HEADER_KEY,
    SESSION_KEY,
    UNKNOWN_CATEGORY,
    DEBUG,
    PRIOR_SESSION_NUM,
)
import new_version.utils as utils
from new_version.DatabaseTypes import DbObject, convertDictToHuman, convertDictToObject, convertHumanToDbQuery, convertObservationToObjectDict, convertObjectToDbQuery
from new_version.MemoryManager import MemoryManager
from Ontology import ontology_member
from new_version.visualisation import RvizVisualisationManager

MAX_DISTANCE_DEFAULT_PARAM = "default"


class PositionalCovariance(Enum):
    B14 = 0
    Running_avg = 1
    No_covariance = 2


class ObjectsManager:
    def __init__(
        self,
        memory_manager: MemoryManager,
        ontology_tree: ontology_member,
        visualisation_manager: Optional[RvizVisualisationManager] = None,
        collection_name="objects",
        is_prior=False,
        sort_queries_by: Optional[str] = None,
        max_distance_same_obj: Union[Dict[str, float], float] = 0.0,
        dont_edit_on_update: Optional[List[str]] = None,
        positional_covariance_type: PositionalCovariance = PositionalCovariance.No_covariance,
        min_observations_for_query = 0
    ):
        """
        This class manages the services for inserting/querying the objects DB.

        Params:
        - memory_manager: the MemoryManger used to connect to the db
        - ontology_tree: used to determine the category of an observation from its class
        - visualization_manager: used to visualize objects added to the SOM
        - collection_name: the name of the DB collection accessed by this class
        - is_prior: if true, the session number will be `constants.PRIOR_SESSION_NUM`
                    instead of being the current session number
        - sort_queries_by: an attribute of DbObject, will be used to sort the queries
        - max_distance_same_obj: the maximum distance from which two objects are considered
                                to be the same. If it is a dict, it should give a distance 
                                for each class_ number, and it must contain `constants.MAX_DISTANCE_DEFAULT_PARAM` 
                                as a default which will be used for the missing classes.
        - dont_edit_on_update: the keys of a DbObject that we do not want to change when updating from a new
                                observation. The default is an empty list.
        - positional_covariance_type: The type of calculation used to approximate the position
        - min_observations_for_query: The min number of observations for an object to be queried from the DB
        """ 
        # TODO: do we need the session num?
        # TOOd
        self.memory_manager = memory_manager
        self.ontology_tree = ontology_tree
        self.visualisation_manager = visualisation_manager
        self.is_prior = is_prior
        self.sort_queries_by = sort_queries_by
        self.max_distance_same_obj = max_distance_same_obj
        self.positional_covariance_type = positional_covariance_type
        self.min_observations_for_query = min_observations_for_query

        self.tf_name_counter: Dict[str, int] = {}
        self.latest_tf_name: str = ""

        self.collection: pymongo.collection.Collection[
            DbObject
        ] = self.memory_manager.getCollection(collection_name)

        self.dont_edit_on_update = (
            [] if dont_edit_on_update is None else dont_edit_on_update
        )

        self.setup_services()

    def setup_services(self):
        rospy.Service(
            SERVICE_ROOT + "observations/input",
            act_srv.SOMAddObservation,
            self.rosAddObservationEntypoint,
        )

        rospy.Service(
            SERVICE_ROOT + "objects/basic_query",
            act_srv.SOMQueryObjects,
            self.rosQueryObjectsEntrypoint,
        )

        rospy.Service(
            SERVICE_ROOT + "humans/basic_query",
            act_srv.SOMQueryHumans,
            self.rosQueryHumansEntrypoint,
        )


    def rosQueryObjectsEntrypoint(self, query: act_srv.SOMQueryObjectsRequest) -> act_srv.SOMQueryObjectsResponse:
        db_query = convertObjectToDbQuery(query.query)

        self.addSessionNum(db_query)
        
        results = self.queryIntoCollection(db_query) # type:ignore
        objects = [convertDictToObject(obj_dict) for obj_dict in results]
        return act_srv.SOMQueryObjectsResponse(returns = objects)


    def rosQueryHumansEntrypoint(self, query: act_srv.SOMQueryHumansRequest) -> act_srv.SOMQueryHumansResponse:
        db_query = convertHumanToDbQuery(query.query)

        self.addSessionNum(db_query)
        
        results = self.queryIntoCollection(db_query) # type:ignore
        humans = [convertDictToHuman(obj_dict) for obj_dict in results]
        return act_srv.SOMQueryHumansResponse(returns = humans)
        

    def rosAddObservationEntypoint(
        self, request: act_srv.SOMAddObservationRequest
    ) -> act_srv.SOMAddObservationResponse:
        """
        Callback for adding a single observation through ROS service.
        """
        uid = self.addObservationToCollection(request.adding)
        return act_srv.SOMAddObservationResponse(UID=uid)

    def addObservationToCollection(
        self, observation: act_msg.SOMObservation
    ) -> ObjectId:
        """
        Add a new observation to the Database, by either creating a new object
        or updating an already existing one.
        """
        obj_to_update = self.getObjectToUpdate(observation)
        
        if obj_to_update is None:
            return self.addNewObjectToCollection(observation)
        
        self.updateObjectWithObsevation(obj_to_update, observation)
        return obj_to_update[PYMONGO_ID_KEY]  # type: ignore

    def updateObjectWithObsevation(
        self, obj_to_update: DbObject, observation: act_msg.SOMObservation
    ):
        """
        Update the given object with the info from the observation.
        """
        # Update the position of the object by computing a mean of all observations so far
        if self.positional_covariance_type == PositionalCovariance.B14:
            # TODO: Reimplement the B14 calculation (probably not used curently?)
            pass
        # Executes a simple average. (To be used if B14 stuff is not to be!)
        elif self.positional_covariance_type == PositionalCovariance.Running_avg:
            avg_pos = utils.getPositionFromDictAsArray(obj_to_update)
            new_pos = np.asarray(
                [
                    observation.obj_position.position.x,
                    observation.obj_position.position.y,
                    observation.obj_position.position.z,
                ]
            )
            num_obs = obj_to_update["num_observations"]  # type: ignore
            new_avg = (avg_pos * num_obs + new_pos) / (num_obs + 1)
            observation.obj_position.position = Point(
                x=new_avg[0], y=new_avg[1], z=new_avg[2]
            )
        # Note that this will set the last_observed_at value to the time of this observation
        prof.start("create-updating-obj-dict")
        new_obj_dict = convertObservationToObjectDict(observation)
        prof.record("create-updating-obj-dict")
        # We remove the information that should not be updated (e.g we do not want to update the first_observed_at)
        for key in self.dont_edit_on_update:
            del new_obj_dict[key]

        new_obj_dict["num_observations"] = obj_to_update["num_observations"] + 1  # type: ignore

        # TODO: Add back the code doing frequency analysis (not used currently)
        uid = obj_to_update[PYMONGO_ID_KEY]  # type: ignore
        prof.start("insert-update")
        self.collection.update_one({PYMONGO_ID_KEY: uid}, {"$set": new_obj_dict})
        prof.record("insert-update")

        if DEBUG_LONG:
            print("Updating ", uid, "with", new_obj_dict)
        elif DEBUG:
            print("Updating ", uid)

        if self.visualisation_manager is not None:
            self.visualisation_manager.add_obj_dict(
                new_obj_dict, uid, new_obj_dict["num_observations"]
            )

    def addNewObjectToCollection(self, observation: act_msg.SOMObservation) -> ObjectId:
        """
        Add the observation to the database, as a new object.
        """
        prof.start("create-new-obj-dict")
        dict_to_add = self.createNewObject(observation)    
        prof.record("create-new-obj-dict")
        if DEBUG_LONG:
            print("Adding an entry to", self.collection.name, "\n\t", dict_to_add, "\n")
        elif DEBUG:
            print("Adding an entry to", self.collection.name)
        prof.start("insert-new-obj-dict")
        result = self.collection.insert_one(dict_to_add)
        prof.record("insert-new-obj-dict")
        result_id: ObjectId = result.inserted_id

        if self.visualisation_manager is not None:
            self.visualisation_manager.add_obj_dict(dict_to_add, result_id, 1)

        return result_id
    
    def createNewObject(self, observation: act_msg.SOMObservation) -> DbObject:
        """
        Create a DbObject from an observation, ready to be added to the Database.

        This function will: 
        - add a a category if missing
        - add a (new) tf_name for the object
        - add the human properties if needed
        - add the session number to the header
        """
        category = observation.category
        if category == "":
            category = self.getObjectCategory(observation.class_)
            if DEBUG:
                print("setting category to", category)

        pickupable = observation.class_ not in NON_PICKUPABLE_OBJS
        tf_name = self.getNewTFName(observation.class_)
        self.latest_tf_name = tf_name
        is_human = observation.class_ == HUMAN_CLASS

        dict_to_add = convertObservationToObjectDict(
            observation, category, pickupable, tf_name, is_human
        )
        self.addSessionNum(dict_to_add)
        return dict_to_add

    def getNewTFName(self, obj_class: str) -> str:
        # TODO: Should this be here?
        if obj_class in self.tf_name_counter:
            self.tf_name_counter[obj_class] += 1
        else:
            self.tf_name_counter[obj_class] = 0
        return obj_class + "_" + str(self.tf_name_counter[obj_class])

    def getObjectCategory(self, obj_class: str):
        ontological_result = self.ontology_tree.search_for_term(obj_class)

        if ontological_result is None:
            self.ontology_tree.add_term([UNKNOWN_CATEGORY, obj_class])
            return UNKNOWN_CATEGORY

        # So this will go [class, category, "Objs"];
        return ontological_result[1]

    def getObjectToUpdate(
        self, observation: act_msg.SOMObservation
    ) -> Optional[DbObject]:
        """
        If the object detected in the observation is already present in  the DB, 
        this function returns it. Otherwise, it returns None.

        This is done by checking if an object with the same class and with 
        (approximately) the same position is already in the database.
        """

        position = np.asarray(
            [
                observation.obj_position.position.x,
                observation.obj_position.position.y,
                observation.obj_position.position.z,
            ]
        )

        query = {"class_": observation.class_}

        # TODO: understand what this does
        # This means we have a greedy implementation that just takes the closest
        # option each time. This is almost certainly fine (bar for really uncertain cases).
        # if self.consistency_args.batch_nums_setup():
        #    query[self.consistency_args.last_observation_batch] = \
        #        {"$lt" : adding[self.consistency_args.observation_batch_num]}

        possible_results = self.queryIntoCollection(query)

        max_distance = self.getMaxDistance(observation.class_)

        # Doing the filtering to work out which object you want to look at (if any).
        obj_to_update: Optional[DbObject] = None
        for element in possible_results:
            element_pos = utils.getPositionFromDictAsArray(element)
            dist = np.linalg.norm(position - element_pos)
            if dist < max_distance:
                obj_to_update = element
                max_distance = dist

        return obj_to_update

    def queryIntoCollection(self, query: Dict[str, Any]) -> List[DbObject]:
        """
        Query into the Database.
        This will return a list of dictionaries, each one corresponding to an entry.
        """

        # TODO: Check that the only query_callback is the num_observation_threshold
        self.setMinNumObservationsInQuery(query)
            
        # Every query must have a session number. If it is not present, we add it manually
        self.addSessionNum(query)

        if DEBUG:
            print("Querying into", self.collection.name, ":\t", query)
        
        prof.start("query-obj")
        query_result = self.collection.find(query)
        prof.record("query-obj")
        query_result_list = list(query_result)

        if self.sort_queries_by is not None:
            query_result_list.sort(key=lambda x: x[self.sort_queries_by], reverse=True)  # type: ignore

        if DEBUG:
            print("\tresponse length =", len(query_result_list))

        return query_result_list
    
    def setMinNumObservationsInQuery(self, query: Dict[str, Any]):
        num_observations_threshold =  self.min_observations_for_query
        if "num_observations"  in query:
            num_observations_threshold = query["num_observations"]
            # WE remove it since we want to add the correct query later
            del query["num_observations"]
        # If the minimum number is 0 or 1, it is useless so we don't add it
        if self.min_observations_for_query > 1:
            query["num_observations"] = {"$gt" : num_observations_threshold}


    def addSessionNum(self, obj_dict: Union[DbObject, Dict[str, Any]]):
        if HEADER_KEY not in obj_dict:
            obj_dict[HEADER_KEY] = {}  # type: ignore

        header = obj_dict[HEADER_KEY]  # type: ignore

        if SESSION_KEY in header:
            return

        header[SESSION_KEY] = (
            PRIOR_SESSION_NUM
            if self.is_prior
            else self.memory_manager.current_session_num
        )

    def getMaxDistance(self, obj_class: str) -> float:
        """
        Returns the maximum distance at which two objects of this class should be
        considered to be the same.

        The max_distance_same_obj can be either a dict or a float. If it is a 
        dict, we want to return the max distance for this class, or the default
        one if a distance for this class is not specified.
        """
        if isinstance(self.max_distance_same_obj, dict):
            if obj_class in self.max_distance_same_obj:
                return self.max_distance_same_obj[obj_class]
            else:
                return self.max_distance_same_obj[MAX_DISTANCE_DEFAULT_PARAM]

        return self.max_distance_same_obj
