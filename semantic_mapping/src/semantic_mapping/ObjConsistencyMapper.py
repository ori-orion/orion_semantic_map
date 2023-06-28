"""
Author: Matthew Munks
Owner: Matthew Munks
"""

import math
import numpy
import utils;
from CollectionManager import CollectionManager, TypesCollection;
import pymongo.collection
import time;

from MemoryManager import DEBUG_LONG;

# This contains all the arguments for checking object consistency.
# So we want to be able to easily identify which attribute
# of an object is the position, which is the size, etc.
# This is a first attempt at defining this.
class ConsistencyArgs:
    DEFAULT_PARAM = "default";

    def __init__(
        self, 
        position_attr=None, 
        size_attr=None,
        max_distance=math.inf,
        first_observed_attr:str=None,
        last_observed_attr:str=None,
        observed_at_attr:str=None,
        observation_batch_num:str=None,
        last_observation_batch:str=None,
        class_identifier:str=None,
        positional_covariance_attr:str=None,
        observation_counter_attr:str=None,
        use_running_average_position:bool=True,
        suppress_double_detections:bool=False,
        suppression_default_distance:float=0,
        suppression_distance_dict:dict=None,
        tf_name_attr:str=None):

        """
        Constructor:
        Inputs:
            position_attr
                The name of the positional attribute in the message file you're reading in. 
            size_attr
                The name of the size attribute in the message file you're reading in. 
            max_distance
                The maximum distance over which consistency checking is performed. This can either be a number, 
                or it can be a dictionary. If it is a dictionary, `default` is a necessary field and gives the 
                default distance for consitency checking. The other keys are those specified by the parameter
                given by `class_identifier`.
            class_identifier:str
                The name of a general identifying parameter within the system that can be used to work out the
                max_distance to use, as well as for suppressing double detections (and probably more to come as 
                well).
        Positional averaging parameters.
            positional_covariance_attr:str
                A matrix (in 1x9 matrix form) for the positional covariance so that we can implement the B14 
                stuff.
            use_running_average_position:bool
                Boolean for activating the simple averaging code. (Will only activate that code if the 
                positional covariance stuff can't be used.)
        Temporal attributes:
            first_observed_attr:str
                Within the consitent object, we need to know which parameter gives the time at which the first
                observation occured.
            last_observed_attr:str
                Within the consitent object, we need to know which parameter gives the time at which the most 
                recent observation occured.
            observed_at_attr:str
                Within the observations, we need to know when it was observed at.
        Batch number attributes.
            observation_batch_num:str
                For the observations, we need to know the observation batch a given observation came from.
            last_observation_batch:str
                Within the objects, we need to know the last batch of observations attributed to that object.            
            observation_counter_attr:str
        Suppression of double detections
            suppress_double_detections:bool
            suppression_default_distance:float
            suppression_distance_dict:dict
        Tf name getting
            tf_name_attr:str
        """


        self.position_attr = position_attr;
        self.positional_covariance_attr = positional_covariance_attr;
        self.size_attr = size_attr;
        
        # If the entry has the covariance stuff then that is used automatically.
        # Otherwise, if the use_running_average_position flag is True, then it will average position over time.
        #               (Useful for stationary objects.)
        #            if the use_running_average_position flag is False, then it will take the most recent position
        #               instead. (Useful for objects moving at high speed.)
        self.use_running_average_position = use_running_average_position;

        self.cross_ref_attr = [];

        # This is currently a simple distance check. Maybe link it to the size of an object?
        self.max_distance = max_distance;
        if type(self.max_distance) is dict:
            if ConsistencyArgs.DEFAULT_PARAM not in self.max_distance:
                self.max_distance[ConsistencyArgs.DEFAULT_PARAM] = math.inf;
        self.class_identifier = class_identifier; 


        # Temporal value names.
        self.first_observed_attr = first_observed_attr;
        self.last_observed_attr = last_observed_attr;
        self.observed_at_attr = observed_at_attr;

        # How many observations of a given entity have been made?
        self.observation_counter_attr = observation_counter_attr;

        # Batch information:
        # Batch number for the things you're pulling from.
        # This is the entry within the observation.
        self.observation_batch_num = observation_batch_num;
        # Latest batch number from the observations for the entity you're pushing to.
        # This is the entry within the consistent object.
        self.last_observation_batch = last_observation_batch;

        # Which attributes don't you want transferred upon an observation?
        self.dont_transfer = [
            observed_at_attr, position_attr, observation_batch_num,
            positional_covariance_attr
        ];

        # The vision system gives bounding boxes for the same object on occaision. This is to stop these double detections
        # from being forwarded onto the SOM system. The default_field argument gives some default parameter for the 
        # suppression distance.
        self.suppress_double_detections = suppress_double_detections;
        self.suppression_default_distance = suppression_default_distance;
        if suppression_distance_dict == None:
            self.suppression_distance_dict = {};
        else:
            self.suppression_distance_dict = suppression_distance_dict;

        # There are some things for which we will want to do a frequency analysis. (Say for instance that you have a 
        # segmentation algorithm and you want to couple object names to the segmented objects. For this, you would
        # want to take all of the classes that the recogniser thinks it might be and then take the most common of those.)
        self.frequency_analysis_attrs = [];

        # If we want the tf name to be stored in self.metadata_latent_adding for the sake of#
        # tf publishing, we need that to get through to the update function.
        self.tf_name_attr = tf_name_attr;

        # -------------------- Not yet implemented --------------------

        # If we are averaging over position, this gives the number of batch numbers we go back by
        # for the averaging process. 
        #   - Is this a simple counter? (If we turn round having not seen the object for average_back_to_batch
        #       batches, are we just going to update its position?)
        self.average_back_to_batch = None;

    def batch_nums_setup(self) -> bool:
        return self.observation_batch_num != None and self.last_observation_batch != None;


# So this is the base layer, that then pushes to pushing_to
class ConsistencyChecker(CollectionManager):
    def __init__(
            self, 
            pushing_to:CollectionManager, 
            types:TypesCollection, 
            service_name:str,
            consistency_args:ConsistencyArgs=ConsistencyArgs(),
            collection_input_callbacks:list=None):
        
        super(ConsistencyChecker, self).__init__(
            types=types, 
            service_name=service_name, 
            memory_manager=pushing_to.memory_manager);

        self.pushing_to = pushing_to;

        # We don't really want hard coded values here, BUT, in order to average over 
        # position in an intelligent way (using \Sigma), we're going to start to need
        # to define parameters. This does this!
        self.consistency_args:ConsistencyArgs = consistency_args;

        if collection_input_callbacks == None:
            collection_input_callbacks = [];
        if self.consistency_args.suppress_double_detections and self.consistency_args.batch_nums_setup():
            collection_input_callbacks.insert(0, self.suppression_callback);
        
        self.collection_input_callbacks = collection_input_callbacks;
        self.collection_input_callbacks.append(self.push_item_to_pushing_to);
        self.pushing_to.collection_query_callbacks.append(self.query_callback);

        self.pushing_to.sort_queries_by = consistency_args.last_observation_batch;

        print(self.service_name, ": max distance =", self.consistency_args.max_distance);
    
    # The function for actually adding something to the other collection.
    def createNewConsistentObj(self, adding:dict) -> str:
        """
        Creates a new consistent object in the entities collection.
        """
        
        # Maybe do some fun stuff looking at size here??
        adding[self.consistency_args.first_observed_attr] = \
            adding[self.consistency_args.observed_at_attr];
        adding[self.consistency_args.last_observed_attr] = \
            adding[self.consistency_args.observed_at_attr];

        if self.consistency_args.batch_nums_setup():
            adding[self.consistency_args.last_observation_batch] = \
                adding[self.consistency_args.observation_batch_num];
            del adding[self.consistency_args.observation_batch_num];

        if self.consistency_args.observation_counter_attr != None:
            adding[self.consistency_args.observation_counter_attr] = 1;

        return str(self.pushing_to.addItemToCollectionDict(adding));

    def updateConsistentObj(self, updating_info:dict, obj_id_to_update:pymongo.collection.ObjectId, num_observations=math.inf):
        """
        Updates an object of uid `obj_id_to_update` with the dictionary `updating_info`.
        This also does the positional updating averaging stuff/carries out the B14 stuff. 
        `num_observations` gives the number of observations for the visualisation system. 
            (There is no other purpose of this here.)
        """
        # updating_info is an observation, rather than an object.

        previously_added:list = None;

        def getPreviouslyAdded():
            if previously_added == None:
                previously_added = self.queryIntoCollection({utils.CROSS_REF_UID: str(obj_id_to_update)});
            return previously_added;

        update_entry_input = {};

        # We want to update everything to match the observtion.
        for key in updating_info:
            # There are however some things we don't want to blindly update.
            if (key not in self.consistency_args.dont_transfer):
                update_entry_input[key] = updating_info[key];

        #region Doing positional averaging
        # Covariance implementation (using the B14 implementation).
        if self.consistency_args.positional_covariance_attr != None and \
            len(updating_info[self.consistency_args.positional_covariance_attr]) == 9:

            means = [];
            covariances = [];
            previously_added = getPreviouslyAdded();
            for element in previously_added:
                means.append(numpy.asarray(utils.getPoint(element[self.consistency_args.position_attr])));
                cov_mat = utils.getMatrix(element[self.consistency_args.positional_covariance_attr]);
                covariances.append(cov_mat);
                #print(cov_mat);

            means.append(numpy.asarray(utils.getPoint(updating_info[self.consistency_args.position_attr])));
            cov_mat = utils.getMatrix(updating_info[self.consistency_args.positional_covariance_attr])
            covariances.append(cov_mat);
            #print(cov_mat);

            updating_info[self.consistency_args.position_attr] = \
                utils.setPoint(updating_info[self.consistency_args.position_attr], utils.get_mean_over_samples(means, covariances));
        # Executes a simple average. (To be used if B14 stuff is not to be!)
        elif self.consistency_args.use_running_average_position:
            points = [];
            previously_added = getPreviouslyAdded();
            point_av = utils.getPoint(updating_info[self.consistency_args.position_attr]);
            num_points = 1;
            for element in previously_added:
                pt = utils.getPoint(element[self.consistency_args.position_attr])
                points.append(pt);
                point_av += pt;
                num_points += 1;
                
            point_av /= num_points;

            updating_info[self.consistency_args.position_attr] = \
                utils.setPoint(updating_info[self.consistency_args.position_attr], point_av);
        #endregion

        update_entry_input[self.consistency_args.position_attr] = \
            updating_info[self.consistency_args.position_attr];

        update_entry_input[self.consistency_args.last_observed_attr] = \
            updating_info[self.consistency_args.observed_at_attr];

        # print("Batch nums set up...", self.consistency_args.batch_nums_setup());
        if self.consistency_args.batch_nums_setup():
            update_entry_input[self.consistency_args.last_observation_batch] = \
                updating_info[self.consistency_args.observation_batch_num];
        
        # Infrastructure for setting up the incrementing of the observations counter.
        increment_param=None;
        if self.consistency_args.observation_counter_attr != None:
            increment_param = {self.consistency_args.observation_counter_attr: 1};

        #region Doing frequency analysis over discrete parameters (like strings).
        for attr in self.consistency_args.frequency_analysis_attrs:
            frequency_keys = {};
            previously_added = getPreviouslyAdded();
            max_freq = 0;
            updating_attr_to = None;
            for result in previously_added:
                result:dict;
                if attr not in result:
                    continue;

                dict_key = str(result[attr]);

                if dict_key in frequency_keys:
                    frequency_keys[dict_key] += 1;
                else:
                    frequency_keys[dict_key] = 1;
                if frequency_keys[dict_key] > max_freq:
                    max_freq = frequency_keys[dict_key];
                    updating_attr_to = result[attr];
            
            update_entry_input[attr] = updating_attr_to;
        #endregion

        if (DEBUG_LONG):
            print(update_entry_input);

        # Update the entry in pushing_to.
        self.pushing_to.updateEntry(obj_id_to_update, update_entry_input, increment_param);

        # Refresh the visualisations.
        if self.pushing_to.visualisation_manager != None:
            self.pushing_to.visualisation_manager.add_obj_dict(updating_info, str(obj_id_to_update), num_observations);


    # Returns the str id that the object has gone into.
    def push_item_to_pushing_to(self, adding:dict, metadata:dict) -> str:
        """
        The callback that adds observations to the object collection.
        This gets the full adding dict (with all default fields).
        It filters objects based on raw distance (rather than anything more fancy) at present.
            This is done to work out whether to update or to create a new entry.
        """
        adding_pos = utils.getPoint(adding[self.consistency_args.position_attr]);
        
        query = {};
        for element in self.consistency_args.cross_ref_attr:
            query[element] = adding[element];
            

        # This means we have a greedy implementation that just takes the closest 
        # option each time. This is almost certainly fine (bar for really uncertain cases).
        if self.consistency_args.batch_nums_setup():
            query[self.consistency_args.last_observation_batch] = \
                {"$lt" : adding[self.consistency_args.observation_batch_num]}

        tic = time.perf_counter();
        possible_results:list = self.pushing_to.queryIntoCollection(query);
        toc = time.perf_counter();
        print("\t\t\tqueryIntoCollection(...) took {0} seconds.".format(toc-tic));

        if len(possible_results) == 0:
            # print("No matches.")
            metadata['obj_uid'] = self.createNewConsistentObj(adding);
            return adding, metadata;

        # print("There were", len(possible_results), "possible matches");
        tic = time.perf_counter();
        # Working out what the max distance should be.
        max_distance = self.consistency_args.max_distance;
        if type(max_distance) is dict and self.consistency_args.class_identifier != None:
            max_distance:dict;
            obj_class = adding[self.consistency_args.class_identifier];
            if obj_class in max_distance:
                max_distance = max_distance[obj_class];
            else:
                max_distance = max_distance[ConsistencyArgs.DEFAULT_PARAM];

        # Doing the filtering to work out which object you want to look at (if any).
        # print("Max distance", max_distance);
        updating = None;
        num_prev_observations = math.inf;
        for element in possible_results:
            element_pos = utils.getPoint(element[self.consistency_args.position_attr]);
            dist = numpy.linalg.norm(adding_pos - element_pos);
            if (dist < max_distance):
                updating = element;
                if self.consistency_args.observation_counter_attr != None:
                    num_prev_observations = element[self.consistency_args.observation_counter_attr];
                max_distance = dist;
                # print("Updating ", element);
        toc = time.perf_counter();
        print("\t\t\tWorking out max distance and updating obj took {0}s".format(toc-tic));


        if (updating == None):
            tic = time.perf_counter();
            metadata['obj_uid'] = self.createNewConsistentObj(adding);
            toc = time.perf_counter();
            print("\t\t\tAdding a new consistent obj took {0}s".format(toc-tic));
            return adding, metadata;
        else:
            if self.consistency_args.tf_name_attr != None and self.consistency_args.tf_name_attr in updating:
                self.pushing_to.metadata_latent_adding = {self.consistency_args.tf_name_attr:updating[self.consistency_args.tf_name_attr]};

            # Update an existing entry.
            tic = time.perf_counter();
            self.updateConsistentObj(adding, updating[utils.PYMONGO_ID_SPECIFIER], num_prev_observations);
            toc = time.perf_counter();
            print("\t\t\tUpdating a consistent obj took {0}s".format(toc-tic));
            metadata['obj_uid'] = str(updating[utils.PYMONGO_ID_SPECIFIER]);
            return adding, metadata;

    
    def suppression_callback(self, adding:dict, metadata:dict):
        """
        This is the callback for the suppression of double detections 
        (where suppression is done by distance).
        """
        # print("Suppression callback")
        if self.consistency_args.class_identifier not in adding:
            return adding, metadata;

        adding_pos = utils.getPoint(adding[self.consistency_args.position_attr]);

        query = {};
        for element in self.consistency_args.cross_ref_attr:
            query[element] = adding[element];

        if adding[self.consistency_args.class_identifier] in self.consistency_args.suppression_distance_dict:
            suppression_distance = self.consistency_args.suppression_distance_dict[adding[self.consistency_args.class_identifier]];
        else:
            suppression_distance = self.consistency_args.suppression_default_distance;

        # For objects really close together, it might be a double detection...
        # It's therefore nice to be able to suppress double detections.
        query[self.consistency_args.last_observation_batch] = adding[self.consistency_args.observation_batch_num];
        possible_results:list = self.pushing_to.queryIntoCollection(query);
        for element in possible_results:
            element_pos = utils.getPoint(element[self.consistency_args.position_attr]);
            dist = numpy.linalg.norm(adding_pos - element_pos);
            if dist < suppression_distance:
                metadata['prevent_from_adding'] = True;
                print("Suppressing a double detection of class ", adding[self.consistency_args.class_identifier]);
                return adding, metadata;

        return adding, metadata;

    def query_callback(self, querying:dict, metadata:dict):
        if self.consistency_args.batch_nums_setup():
            if self.consistency_args.last_observation_batch in querying:
                batch_num_query = querying[self.consistency_args.last_observation_batch];
                if type(batch_num_query) is int and batch_num_query < 0:
                    querying[self.consistency_args.last_observation_batch] = {"$gt" : -batch_num_query};

        if self.consistency_args.last_observed_attr in querying:
            temp_query:dict = querying[self.consistency_args.last_observed_attr];
            # querying[self.consistency_args.last_observed_attr] = {'secs': {'$gte': temp_query['secs'] }}
            del querying[self.consistency_args.last_observed_attr];
            querying[self.consistency_args.last_observed_attr + ".secs"] = {'$gte': temp_query['secs'] };
            
        return querying, metadata;