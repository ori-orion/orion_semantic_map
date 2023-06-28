"""
Author: Matthew Munks
Owner: Matthew Munks
"""

import numpy
import utils;
import pymongo;
import pymongo.collection
import pymongo.cursor
import rospy;
import genpy;
from MemoryManager import UID_ENTRY, MemoryManager, DEBUG, DEBUG_LONG, SESSION_ID, SERVICE_ROOT;
from utils import HEADER_ID, UID_ENTRY;

import visualisation;

import std_srvs.srv;
import time;

from typing import List, Callable, Tuple


class TypesCollection:
    """
    To define all the types we're looking at for the queries.
    """
    def __init__(self, 
        base_ros_type:type, 
        input_parent:type=None,
        input_response:type=None,
        query_parent:type=None, 
        query_response:type=None,
        input_array_parent:type=None,
        input_array_response:type=None):

        self.base_ros_type:type = base_ros_type;

        # Services have the parent type, the request type and the response type. 
        # The request can easily just be read in. That makes the response and the
        # parent the only types of relevance here.
        self.query_parent:type = query_parent;
        self.query_response:type = query_response;

        # The service definition for the input type.
        self.input_parent:type = input_parent;
        self.input_response:type = input_response;

        self.input_array_parent:type = input_array_parent;
        self.input_array_response:type = input_array_response;    


class CollectionManager:
    """
    This will be the collection level interface into pymongo, as well as dealing with the basic
    service queries in and out of the collection.

    types           - The base ROS type that this inherits from, and the derived service types for 
                        querying and adding into the system. Note that the types within this can be
                        None, but the type itself cannot be.
    service_name    - The rossrv list name of the service. This is the service name that you
                        will be able to send stuff to in order to add it to this collection.
                        Note that this will also be the name of the pymongo collection.    
    memory_manager  - The interface through which the basic memory mangement will happen.  
    positional_attr - For relations, we want to know what entry determines the position of an object.
                        This will also be used within the ObjConsistencyChecker.                           
    """

    PRIOR_SESSION_ID = -1;

    def __init__(self, 
        types:TypesCollection, 
        service_name:str, 
        memory_manager:MemoryManager, 
        visualisation_manager:visualisation.RvizVisualisationManager=None,
        sort_queries_by:str=None,
        is_prior:bool=False):

        self.types:TypesCollection = types;
        self.service_name:str = service_name;
        self.memory_manager:MemoryManager = memory_manager;

        # Makes sure the collection is added to the memory manager.
        self.collection:pymongo.collection.Collection = memory_manager.addCollection(self.service_name);        

        # These are all in the form (action_dict, obj_id) -> (action_dict, obj_id), so that they can
        # be called in series. obj_id is for cross referencing, and if it's not None, then 
        # at the end, action_dict[utils.CROSS_REF_UID] will be set to obj_id. 
        # The primary reason for the existance of these is for that of cross referencing between 
        # collections. (Observations get amalgamated into Objects for instance.) 
        # However, note that obj_id will propagate through all callbacks after it's assigned which in
        # itself is quite useful.  
        self.collection_input_callbacks:List[Callable[[dict,dict],Tuple[dict,dict]]] = [];
        # This goes right at the beginning of the query infrastructure. It only takes
        # query_dict:dict as an input/output.
        self.collection_query_callbacks:List[Callable[[dict,dict],Tuple[dict,dict]]] = [];

        # An attribute of a sortable type (such as int or float).
        # This is what we sort entries by for the query return.
        self.sort_queries_by:str = sort_queries_by;

        # Is what we're looking at a prior? (Should all inputs/outputs have a session number of PRIOR_SESSION_ID?)
        self.is_prior:bool = is_prior;

        self.visualisation_manager = visualisation_manager;
        if visualisation_manager != None:
            self.visualisation_manager.query_callback = self.queryIntoCollection;

        # Stores the last metadata produced by the input or query callbacks.
        # Useful for getting information out of the system.
        self.metadata_latent_adding:dict = None;
        self.metadata_latent_querying:dict = None;

        self.setupServices();
        

    def addItemToCollectionDict(self, adding_dict:dict) -> pymongo.collection.ObjectId:
        """
        Add items to the collection via a dictionary.
        """

        # If SESSION_ID == CollectionManager.PRIOR_SESSION_ID, then it's a prior so...
        adding_header:dict = utils.getHeaderInfoDict_nonCrit(adding_dict);
        if (SESSION_ID not in adding_header) or (adding_header[SESSION_ID] != CollectionManager.PRIOR_SESSION_ID):
            if self.is_prior:
                adding_header[SESSION_ID] = CollectionManager.PRIOR_SESSION_ID;
            else:
                adding_header[SESSION_ID] = self.memory_manager.current_session_id;

        tic = time.perf_counter();
        # This is for inserting stuff into the higher level system.
        # If we're cross referencing entries in the dictionary, we're going to need to log this!
        metadata:dict = { 'prevent_from_adding':False };
        print("Length of callbacks: ", len(self.collection_input_callbacks));
        print(self.collection_input_callbacks);
        for i, callback in enumerate(self.collection_input_callbacks):
            tic_2 = time.perf_counter();
            adding_dict, metadata = callback(adding_dict, metadata);
            if metadata["prevent_from_adding"] == True:
                self.metadata_latent_adding = metadata;
                return "";
            toc_2 = time.perf_counter();
            print("\t\tCallback {0} took time {1}".format(i, toc_2-tic_2));
        toc = time.perf_counter();
        print("\tRunning though callbacks took ", toc-tic);

        # If no obj_id was returned from the callback, then we assume there is no cross-referencing
        # and thus nothing to add here!
        if ('obj_uid' in metadata):
            adding_dict[utils.CROSS_REF_UID] = str(metadata['obj_uid']);
        
        self.metadata_latent_adding = metadata;

        if (DEBUG_LONG):
            print("Adding an entry to", self.service_name ,"\n\t", adding_dict, "\n");
        elif(DEBUG):
            print("Adding an entry to", self.service_name);

        result = self.collection.insert_one(adding_dict);

        result_id:pymongo.collection.ObjectId = result.inserted_id;

        if (self.visualisation_manager != None):
            self.visualisation_manager.add_obj_dict(adding_dict, str(result_id), 1);

        return result_id;

    def addItemToCollection(self, adding) -> pymongo.collection.ObjectId:
        """
        Adds or updates entries into a collection. 
        If you are updating an entry (because its UID is defined), this ignores default values.

        Returns the object id of the item added from the current collection, rather than from any potential 
        cross reference. 
        """
        if len(utils.getUIDObj(adding)) > 0:
            updating_dict:dict = utils.obj_to_dict(adding, ignore_default=True);
            uid:pymongo.collection.ObjectId = pymongo.collection.ObjectId(utils.getUIDObj(adding));
            self.updateEntry(uid, updating_dict);
            return uid;
        else:
            adding_dict:dict = utils.obj_to_dict(adding, ignore_default=False);
            return self.addItemToCollectionDict(adding_dict);

    def rosPushToCollection(self, pushing): # -> self.types.input_response
        """
        The entrypoint into adding a single item into the collection.
        """
        pushing_attr = utils.get_attributes(pushing);
        if DEBUG:
            print("Pushing obj to", self.service_name);
        uid = self.addItemToCollection(getattr(pushing, pushing_attr[0]));

        response = self.types.input_response();

        # This needs to have the field UID in it!
        # Note that this is the service response, rather than the message definition.
        try:
            response.UID = str(uid);
        except AttributeError:
            rospy.logerr("UID is not in the response field for the service being used.");
            print();
            raise Exception("UID is not in the response field for the service being used.");
        
        return response;

    def rosPushToCollectionArr(self, pushing_arr):
        """
        The entrypoint into adding multiple items into the collection.
        """
        pushing_attr = utils.get_attributes(pushing_arr);
        if DEBUG:
            print("Pushing obj to", self.service_name);
        pushing_array:list = getattr(pushing_arr, pushing_attr[0]);
        uid_outputs = [];
        for element in pushing_array:
            uid_outputs.append(str(self.addItemToCollection(element)));

        response = self.types.input_array_response();

        # This needs to have the field UID in it!
        # Note that this is the service response, rather than the message definition.
        try:
            response.UIDs = uid_outputs;
        except AttributeError:
            rospy.logerr("UIDs is not in the response field for the service being used.");
            print();
            raise Exception("UIDs is not in the response field for the service being used.");
        
        return response;


    def updateEntry(self, uid:pymongo.collection.ObjectId, update_to:dict, increment:dict=None):
        """
        https://www.w3schools.com/python/python_mongodb_update.asp

        So we can use
        set = { "$set": { "address": "Canyon 123" } }
        to set the address of the entry to.

        My guess is that you can update the entire entry by simply having
        set = { "address": "Canyon 123" }
        but this is a guess at present.

        Note also, _id is an internal mongodb convention
        """

        self.collection.update_one(
            {utils.PYMONGO_ID_SPECIFIER:uid}, 
            { "$set": update_to}
        );


        if increment != None:
            self.collection.update_one(
                {utils.PYMONGO_ID_SPECIFIER:uid},
                { "$inc": increment});
        
        if (DEBUG_LONG):
            print("Updating ", uid, "within", self.service_name, "with", update_to);
        elif (DEBUG):
            print("Updating ", uid, "within", self.service_name);


    def queryIntoCollection(self, query_dict:dict) -> list:
        """
        Query into the system through a dictionary.
        This will return a list of dictionaries, each one corresponding to an entry.
        """
        
        metadata = {};
        for callback in self.collection_query_callbacks:
            query_dict, metadata = callback(query_dict, metadata);
        
        self.metadata_latent_querying = metadata;

        query_header:dict = utils.getHeaderInfoDict_nonCrit(query_dict);
        if SESSION_ID not in query_header:
            if self.is_prior:
                query_header[SESSION_ID] = CollectionManager.PRIOR_SESSION_ID;
            else:
                query_header[SESSION_ID] = self.memory_manager.current_session_id;

        if (DEBUG):
            print("Querying into", self.service_name, ":\t", query_dict);

        query_result:pymongo.cursor.Cursor = self.collection.find(query_dict);
        query_result_list = list(query_result);

        #print("\tself.sort_queries_by=", self.sort_queries_by);
        if self.sort_queries_by != None:
            # print("\tSorting results by", self.sort_queries_by);
            query_result_list.sort(key=lambda x:x[self.sort_queries_by], reverse=True);

        if DEBUG:
            print("\tresponse length =", len(query_result_list));

        return query_result_list;

    def rosQueryEntrypoint(self, ros_query):    # -> self.types.query_response
        """
        Query into the database via a ROS query.

        Translates the ROS query into a dictionary, and then gets the list response using self.queryIntoCollection(...)
        """

        ros_query_dict:dict = utils.obj_to_dict(
            ros_query,
            ignore_default=True,
            # ignore_of_type=[rospy.Time, rospy.Duration, genpy.rostime.Time],
            convert_caps=True
        );

        # This is currently in the srv Request bit. We need to look in one level. 
        # If all the fields are their default values, then no query will be generated, thus 
        # causing the statement in the else statement to fail. Hence, we need this condition.
        # We are assuming there is only one request field here!
        if (len(ros_query_dict.keys()) == 0):
            ros_query_dict = {};
        else:
            ros_query_dict = ros_query_dict[list(ros_query_dict.keys())[0]];
            # Getting rid of list queries.
            # NOTE: May well want to get rid of this at some point.
            keys_deleting = [];
            for key in ros_query_dict:
                if type(ros_query_dict[key]) is list:
                    keys_deleting.append(key);
            for key in keys_deleting:
                del ros_query_dict[key];


        # Enabling the querying by UID.
        if (HEADER_ID in ros_query_dict) and (UID_ENTRY in ros_query_dict[HEADER_ID]):
            ros_query_dict[utils.PYMONGO_ID_SPECIFIER] = pymongo.collection.ObjectId(ros_query_dict[HEADER_ID][UID_ENTRY]);
            # We need to delete the UID_ENTRY because otherwise it will actually be used in the query!
            del ros_query_dict[HEADER_ID][UID_ENTRY];
        
        response:list = self.queryIntoCollection(ros_query_dict);

        ros_response = self.types.query_response();
        query_response_attr = utils.get_attributes(ros_response)[0];

        resp_array:list = getattr(ros_response, query_response_attr);

        # assert(type(resp_array) is list);

        for element in response:
            if DEBUG_LONG:
                print("Query response: \n\t",element);

            element_header:dict = utils.getHeaderInfoDict_nonCrit(element);
            element_header[UID_ENTRY] = str(element[utils.PYMONGO_ID_SPECIFIER]);

            appending = self.types.base_ros_type();
            appending = utils.dict_to_obj(element, appending);
            resp_array.append(appending);

        return ros_response;


    def deleteAllEntries(self):
        self.collection.delete_many({});

    def rosDeleteAllEntries(self, srv_input:std_srvs.srv.EmptyRequest):
        self.deleteAllEntries();
        print("Deleting all entries within", self.service_name);
        return std_srvs.srv.EmptyResponse();


    def setupServices(self):
        """
        Setup all the services associated with this object.
        """

        if (self.types.input_parent != None):
            rospy.Service(
                SERVICE_ROOT + self.service_name + '/input', 
                self.types.input_parent, 
                self.rosPushToCollection);

        if (self.types.input_array_parent != None):
            rospy.Service(
                SERVICE_ROOT + self.service_name + '/input_array', 
                self.types.input_array_parent, 
                self.rosPushToCollectionArr);
        
        if (self.types.query_parent != None and self.types.query_response != None):
            rospy.Service(
                SERVICE_ROOT + self.service_name + '/basic_query',
                self.types.query_parent,
                self.rosQueryEntrypoint);

        rospy.Service(
            SERVICE_ROOT + self.service_name + '/delete_entries',
            std_srvs.srv.Empty,
            self.rosDeleteAllEntries);

        pass;