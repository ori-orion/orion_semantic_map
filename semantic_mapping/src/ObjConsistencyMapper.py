import math
import numpy;
import utils;
from MemoryManager import CROSS_REF_UID;
from CollectionManager import CollectionManager, TypesCollection;
import pymongo.collection

# This contains all the arguments for checking object consistency.
# So we want to be able to easily identify which attribute
# of an object is the position, which is the size, etc.
# This is a first attempt at defining this.
class ConsistencyArgs:
    def __init__(self, position_attr=None, size_attr=None):
        self.position_attr = position_attr;
        self.size_attr = size_attr;

        self.cross_ref_attr = [];

        # This is currently a simple distance check. Maybe link it to the size of an object?
        self.max_distance = math.inf;


# So this is the base layer, that then pushes to pushing_to
class ConsistencyChecker(CollectionManager):
    def __init__(
            self, 
            pushing_to:CollectionManager, 
            types:TypesCollection, 
            service_name:str,
            consistency_args:ConsistencyArgs=ConsistencyArgs()):
        
        super(ConsistencyChecker, self).__init__(
            types=types, 
            service_name=service_name, 
            memory_manager=pushing_to.memory_manager);

        self.pushing_to = pushing_to;

        # We don't really want hard coded values here, BUT, in order to average over 
        # position in an intelligent way (using \Sigma), we're going to start to need
        # to define parameters. This does this!
        self.consistency_args = consistency_args;

        self.collection_input_callbacks.append(self.push_item_to_pushing_to);

    # The point here is that we could get either a pose or a point as input. 
    # We therefore want to separate these out.
    def getPoint(self, obj:dict) -> numpy.array:
        if "position" in obj:
            obj = obj["position"];
        return numpy.asarray([obj["x"], obj["y"], obj["z"]]);
    def setPoint(self, obj:dict, new_pt:numpy.array) -> dict:
        obj_nested = obj["position"] if ("position" in obj) else obj;
        obj_nested['x'] = new_pt[0];
        obj_nested['y'] = new_pt[1];
        obj_nested['z'] = new_pt[2];
        return obj;
    
    # The function for actually adding something to the other collection.
    def createNewConsistentObj(self, adding:dict) -> str:
        
        # Maybe do some fun stuff looking at size here??

        return str(self.pushing_to.addItemToCollectionDict(adding));

    def updateConsistentObj(self, updating_info:dict, obj_id_to_update:pymongo.collection.ObjectId):

        previously_added:list = self.queryIntoCollection({CROSS_REF_UID, obj_id_to_update})

        points = [];
        point_av = self.getPoint(updating_info[self.consistency_args.position_attr]);
        num_points = 1;
        for element in previously_added:
            points.append(self.getPoint(element[self.consistency_args.position_attr]));
            point_av += points[len(points) - 1];
            num_points += 1;
            
        point_av /= num_points;

        updating_info[self.consistency_args.position_attr] = \
            self.setPoint(updating_info[self.consistency_args.position_attr], point_av);

        self.pushing_to.updateEntry(obj_id_to_update, updating_info);
        pass;


    # Returns the str id that the object has gone into.
    def push_item_to_pushing_to(self, adding:dict) -> str:
        
        query = {};
        for element in self.consistency_args.cross_ref_attr:
            query[element] = adding[element];

        possible_results:list = self.queryIntoCollection(query);

        if len(possible_results) == 0:
            return self.createNewConsistentObj(adding);

        max_distance = self.consistency_args.max_distance;
        adding_pos = self.getPoint(adding[self.consistency_args.position_attr]);
        updating = None;
        for element in possible_results:
            element_pos = self.getPoint(element[self.consistency_args.position_attr]);
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