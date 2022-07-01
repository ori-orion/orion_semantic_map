import math
import numpy
import utils;
from CollectionManager import CollectionManager, TypesCollection;
import pymongo.collection

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
        observation_counter_attr:str=None
        use_running_average_position:bool=True):


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
        self.observation_batch_num = observation_batch_num;
        # Latest batch number from the observations for the entity you're pushing to.
        self.last_observation_batch = last_observation_batch;

        # Which attributes don't you want transferred upon an observation?
        self.dont_transfer = [
            observed_at_attr, position_attr, observation_batch_num,
            positional_covariance_attr
        ];

        # Not yet implemented
        self.average_back_to_batch = 0;

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
            collection_input_callbacks:list = []):
        
        super(ConsistencyChecker, self).__init__(
            types=types, 
            service_name=service_name, 
            memory_manager=pushing_to.memory_manager);

        self.pushing_to = pushing_to;

        # We don't really want hard coded values here, BUT, in order to average over 
        # position in an intelligent way (using \Sigma), we're going to start to need
        # to define parameters. This does this!
        self.consistency_args:ConsistencyArgs = consistency_args;

        self.collection_input_callbacks = collection_input_callbacks;
        self.collection_input_callbacks.append(self.push_item_to_pushing_to);

        self.pushing_to.sort_queries_by = consistency_args.last_observation_batch;

        print(self.service_name, ": max distance =", self.consistency_args.max_distance);
    
    # The function for actually adding something to the other collection.
    def createNewConsistentObj(self, adding:dict) -> str:
        
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
        # updating_info is an observation, rather than an object.

        previously_added:list = self.queryIntoCollection({utils.CROSS_REF_UID: str(obj_id_to_update)})        
        update_entry_input = {};

        # We want to update everything to match the observtion.
        for key in updating_info:
            # There are however some things we don't want to blindly update.
            if (key not in self.consistency_args.dont_transfer):
                update_entry_input[key] = updating_info[key];

        if self.consistency_args.positional_covariance_attr != None and \
            len(updating_info[self.consistency_args.positional_covariance_attr]) == 9:

            # Does the B14 stuff.

            means = [];
            covariances = [];
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
        elif self.consistency_args.use_running_average_position:
            # Executes a simple average. (To be used if B14 stuff is not to be!)
            points = [];
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

        update_entry_input[self.consistency_args.position_attr] = \
            updating_info[self.consistency_args.position_attr];

        update_entry_input[self.consistency_args.last_observed_attr] = \
            updating_info[self.consistency_args.observed_at_attr];

        print("Batch nums set up...", self.consistency_args.batch_nums_setup());
        if self.consistency_args.batch_nums_setup():
            update_entry_input[self.consistency_args.last_observation_batch] = \
                updating_info[self.consistency_args.observation_batch_num];
        
        increment_param=None;
        if self.consistency_args.observation_counter_attr != None:
            increment_param = {self.consistency_args.observation_counter_attr: 1};

        if (DEBUG_LONG):
            pass;
            print(update_entry_input);

        self.pushing_to.updateEntry(obj_id_to_update, update_entry_input, increment_param);

        if self.pushing_to.visualisation_manager != None:
            self.pushing_to.visualisation_manager.add_obj_dict(updating_info, str(obj_id_to_update), num_observations);


    # Returns the str id that the object has gone into.
    def push_item_to_pushing_to(self, adding:dict, obj_id:str) -> str:
        
        query = {};
        for element in self.consistency_args.cross_ref_attr:
            query[element] = adding[element];

        # This means we have a greedy implementation that just takes the closest 
        # option each time. This is almost certainly fine (bar for really uncertain cases).
        if self.consistency_args.batch_nums_setup():
            query[self.consistency_args.last_observation_batch] = \
                {"$lt" : adding[self.consistency_args.observation_batch_num]}

        possible_results:list = self.pushing_to.queryIntoCollection(query);

        if len(possible_results) == 0:
            # print("No matches.")
            return adding, self.createNewConsistentObj(adding);

        # print("There were", len(possible_results), "possible matches");

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
        print("Max distance", max_distance);
        adding_pos = utils.getPoint(adding[self.consistency_args.position_attr]);
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


        if (updating == None):
            return adding, self.createNewConsistentObj(adding);
        else:
            # Update an existing entry.
            self.updateConsistentObj(adding, updating[utils.PYMONGO_ID_SPECIFIER], num_prev_observations);
            return adding, str(updating[utils.PYMONGO_ID_SPECIFIER]);

    pass;
