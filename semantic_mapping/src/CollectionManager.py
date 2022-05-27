import numpy
import utils;
import pymongo;
import pymongo.collection
import rospy;
import genpy;
from MemoryManager import UID_ENTRY, MemoryManager, DEBUG, SESSION_ID;

from geometry_msgs.msg import Pose, Point

# The only reference to the outside system, but it has to be done to
# make it work :(. 
from orion_actions.msg import Relation;

# The root for all things som related.
SERVICE_ROOT = "som/";

class TypesCollection:
    """
    To define all the types we're looking at for the queries.
    """
    def __init__(self, 
        base_ros_type:type, 
        input_parent:type=None,
        input_response:type=None,
        query_parent:type=None, 
        query_response:type=None):

        self.base_ros_type:type = base_ros_type;

        # Services have the parent type, the request type and the response type. 
        # The request can easily just be read in. That makes the response and the
        # parent the only types of relevance here.
        self.query_parent:type = query_parent;
        self.query_response:type = query_response;

        # The service definition for the input type.
        self.input_parent:type = input_parent;
        self.input_response:type = input_response;


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
    def __init__(self, types:TypesCollection, service_name:str, memory_manager:MemoryManager, positional_attr=None):
        self.types:TypesCollection = types;
        self.service_name:str = service_name;        
        self.memory_manager:MemoryManager = memory_manager;

        # Makes sure the collection is added to the memory manager.
        self.collection:pymongo.collection.Collection = memory_manager.addCollection(self.service_name);        

        self.collection_input_callbacks = [];

        self.positional_attr = positional_attr;

        self.setupServices();


    def addItemToCollectionDict(self, adding_dict:dict) -> pymongo.collection.ObjectId:
        """
        Add items to the collection via a dictionary.
        """

        adding_dict[SESSION_ID] = self.memory_manager.current_session_id;

        if (DEBUG):
            print("Adding an entry to", self.service_name ,"\n\t", adding_dict, "\n");

        # This is for inserting stuff into the higher level system.
        # If we're cross referencing entries in the dictionary, we're going to need to log this!        
        obj_id = None;
        for callback in self.collection_input_callbacks:
            obj_id = callback(adding_dict);

        # If no obj_id was returned from the callback, then we assume there is no cross-referencing
        # and thus nothing to add here!
        if (obj_id != None):
            adding_dict[utils.CROSS_REF_UID] = str(obj_id);

        result = self.collection.insert_one(adding_dict);

        result_id:pymongo.collection.ObjectId = result.inserted_id;
        # print(result_id);
        return result_id;

    def addItemToCollection(self, adding) -> pymongo.collection.ObjectId:
        adding_dict:dict = utils.obj_to_dict(adding, ignore_default=False);
        return self.addItemToCollectionDict(adding_dict);

    def rosPushToCollection(self, pushing): # -> self.types.input_response
        pushing_attr = utils.get_attributes(pushing);        
        if DEBUG:
            print("Pushing obj to", self.service_name);
        uid = self.addItemToCollection(getattr(pushing, pushing_attr[0]));

        response = self.types.input_response();
        # this needs to have the field UID in it!
        response.UID = str(uid);
        return response;


    def updateEntry(self, uid:pymongo.collection.ObjectId, update_to:dict):
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
        
        if (DEBUG):
            print("Updating ", uid, "within", self.service_name, "with", update_to);


    def queryIntoCollection(self, query_dict) -> list:
        """
        Query into the system through a dictionary.
        """
        # query_dict = utils.obj_to_dict(query, ignore_default=True);

        if (DEBUG):
            print("Querying into", self.service_name, ":\t", query_dict);

        if SESSION_ID not in query_dict:
            query_dict[SESSION_ID] = self.memory_manager.current_session_id;

        query_result:pymongo.cursor.Cursor = self.collection.find(query_dict);
        query_result_list = list(query_result);

        return query_result_list;

    def rosQueryEntrypoint(self, ros_query):    # -> self.types.query_response
        """
        Query into the database via a ROS query.

        Translates the ROS query into a dictionary, and then gets the list response using self.queryIntoCollection(...)
        """
        ros_query_dict:dict = utils.obj_to_dict(
            ros_query, 
            ignore_default=True,
            ignore_of_type=[rospy.Time, rospy.Duration, genpy.rostime.Time]
        );

        response:list = self.queryIntoCollection(ros_query_dict[list(ros_query_dict.keys())[0]]);

        ros_response = self.types.query_response();
        query_response_attr = utils.get_attributes(ros_response)[0];

        resp_array = getattr(ros_response, query_response_attr);

        assert(type(resp_array) is list);

        for element in response:
            if DEBUG:
                print("Query response: \n\t",element);

            element[UID_ENTRY] = str(element[utils.PYMONGO_ID_SPECIFIER]);

            appending = self.types.base_ros_type();
            appending = utils.dict_to_obj(element, appending);
            resp_array.append(appending);

        return ros_response;        

    
    def get_relation_dict(self, cur_robot_pose:Pose, obj1:dict, obj2:dict) -> Relation:
        """
        HEAVILY inspired by Mark Richter's relational code from the old system.
        """

        dist_thr = 2;

        output_relation:Relation = Relation();
        robot_pos = utils.getPoint(utils.obj_to_dict(cur_robot_pose.position));
        obj_one_pos = utils.getPoint(obj1[self.positional_attr]);
        obj_two_pos = utils.getPoint(obj2[self.positional_attr]);

        print(robot_pos);
        print(obj_one_pos);
        print(obj_two_pos);

        robot_to_two = obj_two_pos - robot_pos
        two_to_one = obj_one_pos - obj_two_pos

        # if the distance between the two objects is greater than the threshold then none of the relations are true
        if numpy.linalg.norm(obj_one_pos - obj_two_pos) > dist_thr:
            output_relation.not_near = True
            return output_relation
        else:
            output_relation.near = True
        # near, not_near set.

        # vertical relation
        eps = 0.001 # use tolerance otherwise comparing relations of object to itself returns true for some fields
        if obj_one_pos[2] > obj_two_pos[2] + eps:
            output_relation.above = True
        elif obj_one_pos[2] < obj_two_pos[2] - eps:
            output_relation.below = True
        # near, not_near, above, below set.

        # forwards and backwards
        if numpy.dot(robot_to_two, two_to_one) > eps:
            output_relation.behind = True
        elif numpy.dot(robot_to_two, two_to_one) < -eps:
            output_relation.frontof = True
        # near, not_near, above, below, behind, frontof set.

        # left and right
        cross_pr = numpy.cross(robot_to_two, two_to_one)
        if cross_pr[2] > eps:
            output_relation.left = True
        elif cross_pr[2] < -eps:
            output_relation.right = True
        # near, not_near, above, below, behind, frontof, left, right set.
        
        # All that is now needed is left_most and right_most, 
        # but that won't happen here (because we're only looking at two objects, not
        # the whole set).

        return output_relation        


    def setupServices(self):
        """
        Setup all the services associated with this object.
        """

        if (self.types.input_parent != None):
            rospy.Service(
                SERVICE_ROOT + self.service_name + '/input', 
                self.types.input_parent, 
                self.rosPushToCollection);
        
        if (self.types.query_parent != None and self.types.query_response != None):
            rospy.Service(
                SERVICE_ROOT + self.service_name + '/basic_query',
                self.types.query_parent,
                self.rosQueryEntrypoint);

        pass;