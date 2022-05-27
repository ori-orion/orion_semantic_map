import math
import numpy;
import utils;
from CollectionManager import CollectionManager, TypesCollection;
import pymongo.collection

# This contains all the arguments for checking object consistency.
# So we want to be able to easily identify which attribute
# of an object is the position, which is the size, etc.
# This is a first attempt at defining this.
class ConsistencyArgs:
    def __init__(self, size_attr=None):        
        self.size_attr = size_attr;

        self.cross_ref_attr = [];

        # This is currently a simple distance check. Maybe link it to the size of an object?
        self.max_distance = math.inf;

        # Temporal value names.
        self.first_observed_attr = None;
        self.last_observed_attr = None;
        self.observed_at_attr = None;

        # Not yet implemented
        self.average_back_to_batch = 0;

        self.positional_covariance_attr = None;


# So this is the base layer, that then pushes to pushing_to
class ConsistencyChecker(CollectionManager):
    def __init__(
            self, 
            pushing_to:CollectionManager, 
            types:TypesCollection, 
            service_name:str,
            consistency_args:ConsistencyArgs=ConsistencyArgs(),
            positional_attr=None):
        
        super(ConsistencyChecker, self).__init__(
            types=types, 
            service_name=service_name, 
            memory_manager=pushing_to.memory_manager,
            positional_attr=positional_attr);

        self.pushing_to = pushing_to;

        # We don't really want hard coded values here, BUT, in order to average over 
        # position in an intelligent way (using \Sigma), we're going to start to need
        # to define parameters. This does this!
        self.consistency_args = consistency_args;

        self.collection_input_callbacks.append(self.push_item_to_pushing_to);
    
    # The function for actually adding something to the other collection.
    def createNewConsistentObj(self, adding:dict) -> str:
        
        # Maybe do some fun stuff looking at size here??
        adding[self.consistency_args.first_observed_attr] = \
            adding[self.consistency_args.observed_at_attr];
        adding[self.consistency_args.last_observed_attr] = \
            adding[self.consistency_args.observed_at_attr];

        return str(self.pushing_to.addItemToCollectionDict(adding));

    def updateConsistentObj(self, updating_info:dict, obj_id_to_update:pymongo.collection.ObjectId):        
        # updating_info is an observation, rather than an object.
        # Thus self.positional_attr is the correct system to use for cross referencing positional
        # information within updating_info.

        previously_added:list = self.queryIntoCollection({utils.CROSS_REF_UID: str(obj_id_to_update)})        

        points = [];
        point_av = utils.getPoint(updating_info[self.positional_attr]);
        num_points = 1;
        for element in previously_added:
            pt = utils.getPoint(element[self.positional_attr])
            points.append(pt);
            point_av += pt;
            num_points += 1;
            
        point_av /= num_points;

        updating_info[self.positional_attr] = \
            utils.setPoint(updating_info[self.positional_attr], point_av);

        update_entry_input = {};
        update_entry_input[self.pushing_to.positional_attr] = \
            updating_info[self.positional_attr];

        update_entry_input[self.consistency_args.last_observed_attr] = \
            updating_info[self.consistency_args.observed_at_attr];

        self.pushing_to.updateEntry(obj_id_to_update, update_entry_input);


    # Returns the str id that the object has gone into.
    def push_item_to_pushing_to(self, adding:dict) -> str:
        
        query = {};
        for element in self.consistency_args.cross_ref_attr:
            query[element] = adding[element];

        possible_results:list = self.pushing_to.queryIntoCollection(query);

        if len(possible_results) == 0:
            # print("No matches.")
            return self.createNewConsistentObj(adding);

        # print("There were", len(possible_results), "possible matches");

        max_distance = self.consistency_args.max_distance;
        # print("Max distance", max_distance);
        adding_pos = utils.getPoint(adding[self.positional_attr]);
        updating = None;
        for element in possible_results:
            element_pos = utils.getPoint(element[self.positional_attr]);
            dist = numpy.linalg.norm(adding_pos - element_pos);
            if (dist < max_distance):
                updating = element;
                max_distance = dist;


        if (updating == None):
            return self.createNewConsistentObj(adding);
        else:
            # Update an existing entry.
            self.updateConsistentObj(adding, updating[utils.PYMONGO_ID_SPECIFIER]);
            return str(updating[utils.PYMONGO_ID_SPECIFIER]);

    pass;