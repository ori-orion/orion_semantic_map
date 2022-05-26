import math
import numpy;
from CollectionManager import CollectionManager, TypesCollection;

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


    # The function for actually adding something to the other collection.
    def createNewConsistentObj(self, adding:dict):
        
        # Maybe do some fun stuff looking at size here??

        self.addItemToCollectionDict(adding);


    def push_item_to_pushing_to(self, adding:dict):
        
        query = {};
        for element in self.consistency_args.cross_ref_attr:
            query[element] = adding[element];

        possible_results:list = self.queryIntoCollection(query);

        if len(possible_results) == 0:
            self.createNewConsistentObj(adding);
            return;

        # The point here is that we could get either a pose or a point as input. 
        # We therefore want to separate these out.
        def getPoint(obj:dict) -> numpy.array:
            if "position" in obj:
                obj = obj["position"];
            return numpy.asarray([obj["x"], obj["y"], obj["z"]]);            

        max_distance = self.consistency_args.max_distance;
        adding_pos = adding[self.consistency_args.position_attr];
        updating = None;
        for element in possible_results:
            element_pos = element[self.consistency_args.position_attr];
            dist = numpy.linalg.norm(adding_pos - element_pos);
            if (dist < max_distance):
                updating = element;
                max_distance = dist;

        if (updating == None):
            self.createNewConsistentObj(adding);
        else:
            # Update an existing entry.
            pass;

        pass;

    pass;