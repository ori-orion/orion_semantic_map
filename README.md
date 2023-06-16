# orion_semantic_map
ORIon Semantic Mapping.

READMEs are included in each subfolder.

Current active branch is `noetic`.

# Notes on current implementation...

## ROS message definitions:

Running the overall system and `rosservice list | grep som` returns 
```
/som/delete_databases
/som/human_observations/delete_entries
/som/human_observations/input
/som/humans/basic_query
/som/humans/delete_entries
/som/object_regions/basic_query
/som/object_regions/delete_entries
/som/object_regions/input
/som/object_regions/region_query
/som/objects/basic_query
/som/objects/delete_entries
/som/objects/input
/som/objects/relational_query
/som/observations/basic_query
/som/observations/delete_entries
/som/observations/input
/som/observations/input_array
```

Note there is some logic to this. First, `{objects, observations, human_observations, humans, object_regions}` refers to different collections in the system. This tells you which collection you're adding to or querying against. Then there are the final addresses of `{input, input_array, basic_query, delete_entries, relational_query, region_query}`. These are worth going into in more detail:

### input
This is for inputting to a collection. The only input field is the SOM message you're inputting into the collection.

### input_array
This is for inputting multiple objects into a collection at the same time. The input field is an array of the message type.

### basic_query
This is for performing basic queries into the system. The input is a query of the given SOM message type. You set all string/numerical/boolean fields you want to query against, and it will match any of these. Any fields that are at their default values won't be queried against. (Note that for boolean fields, this is `False` which is interesting!)

### delete_entries
This is a query of type `std_srvs.srv.Empty` and simply deletes all entries from the system.

### relational_query
This is specific to the objects query. This is for querying for instances where objects matching the query `obj1` are of the relation `relation` with respect to `obj2`. This then returns an array of matches with the responses to `obj1`, `obj2`, `relation` as well as the distance these objects are away from each other.

This was based on Marc Rigter's original SOM code.

### region_query
Regions are defined as priors within the SOM system. (They therefore have a session number of -1). They also have names. This is for querying against a given template (as per basic_query) but also within a region of name `region_name`. This will then return a list of objects within that region.

### objects/basic_query notes

So there are multiple things to note here. Consistent objects have the following properties if relevant (all of which apply to the objects collection):
 - Batch number queries can be for specific batches (if the number is positive) or it can query a range backwards from the latest batch number (if the number is negative). An example of this can be found within tests.py::test_observation_batch_query().
 - Queries into the last_observed_at field always look for objects later than the value given. (Note that the query is rounded down to look only at the seconds). An example of this can be found within tests.py::test_temporal_queries().

# Notes:

## For the addition of collections into the system in general.
The whole aim of the new system is to generalise the memory system s.t. it can be used as a memory system for any generic message. This manifests itself as being able to take an arbitrary ROS message and translate this into a dict for storage in a mongodb. There are a few conventions that need to be satisifed here.
 - Constants are conventionally all uppercase. The system currently checks only the leading character for the identification of constants. Therefore all parameters for storage in the memory system need to start with a lower case.
 - Leading underscores are how the system detects for the parameters in every python type (such as `__dict__`). Thus leading underscores are used for enumerated types within SOM messages. (If this convention is not used, enums within the message types will find their way into the query messages, thus rendering the returns empty.)
 - All entries need a `HEADER` entry of type `SOMHeader.msg`. This then has the `UID` and `SESSION_NUM` entries encoded within it.
    - Priors will be stored with `SESSION_NUM=-1`.

What does this actually mean? You can change the ROS message types, and so long as you don't change something the system actually cares about (such as the positional or batch num parameters), it will read any new/changed fields in automatically. This makes this system versatile and easy to extend to store more types of thing. 

## For looking for the consistency of an object.

 - We want to keep this as abstract and general as possible.
 - We need some sort of observer functionality on additions into the collection you want to do the consistency checking for.
   - Note that we'll call the collection we're cross-referencing `based_off`.
   - We will use a callback function within CollectionManager to then add to the ConsistencyChecker.

### Notes on the implementation of distance checking within ObjConsistencyMapper.py.
So we want something that can be object specific, but also something with a default case. I therefore propose the following: 
 - ConsistencyArgs::max_distance can simply be inputted as a number.
 - ConsistencyArgs::max_distance can also be a dictionary.
   - If it's of type dictionary, then it must have a field max_distance["default"].
      - If it doesn't, this should be automatically filled with math.inf.
      - This will be the field that gets used in the most general case.
   - Otherwise, max_distance[class_identifier] will be used to obtain the minimum consistent distance, where class_identifier is yet another field to set in main.py.

### Notes on how to add separate to the pymongo database from other ROS nodes.

So `semantic_mapping/src/semantic_mapping/MemoryManager.py` is a class that interfaces with the pymongo server. We start this server by running `mongod --port 62345 --dbpath /home/$USER/orion_ws/db`. 

```
# Get a collection of a given name.
def getCollection(memory_manager:MemoryManager, collection_name:str) -> pymongo.collection.Collection:
   return memory_manager.addCollection(collection_name);

# Inserts inserting into the database and returns the _id field. 
def insert_into_collection(collection:pymongo.collection.Collection, inserting:dict):
   result = collection.insert_one(inserting);
   return result.inserted_id;
```

For objects in the SOM system, at present the dictionary form for adding an object can be found below.
```
{
	"HEADER": {
		"SESSION_NUM": [int - don't worry about this for now],
		"UID": [int - Leave blank when adding. Might need to get rid of this...]
	},
	"class_":[str - Object type e.g., "bottle", "cup",...],
	"colour":[str - Less important for now],
	"last_observation_batch":[uint16 - Assuming the observations come in batches, this will be the batch number of the last time you saw an object],
	"num_observations": [uint - Number of times you've seen a given object],
	"category": [str - Will explain when we get to it. Should be easy to implement.]
	"obj_position" : {	// A vec3 and a quaternion... it's in this form because of how the Mem system reads the entries and encodes to a message type.
		"position" : {
			"x" : [float],
			"y" : [float],
			"z" : [float]
		},
		"orientation": {
			"x" : [float],
			"y" : [float],
			"z" : [float],
			"w" : [float]}
	"pickupable" : [bool - Self explanatory, not necessary for the moment],
	"picked_up" : [bool - False for your purposes],
	"first_observed_at" : [time - When did you first observe an object],
	"last_observed_at" : [time - Time of most recent observation],
	"size" : {		// Might be able to do something more complex here given segmentation stuff, but we also want efficiency.
		"x" : [float],
		"y" : [float],
		"z" : [float]
	}
}
```

# Examples for current implementation.

## Querying for all objects.

```
rospy.wait_for_service('/som/objects/basic_query');
object_query_srv = rospy.ServiceProxy('/som/objects/basic_query', SOMQueryObjects);
query = SOMQueryObjectsRequest();
result:SOMQueryObjectsResponse = object_query_srv(query);
print(result);
```
`result` then has the attribute `result.returns:List[SOMObject]` which gives all the objects ever seen.

## Querying for an object of a given class.

```
rospy.wait_for_service('/som/objects/basic_query');
object_query_srv = rospy.ServiceProxy('/som/objects/basic_query', SOMQueryObjects);
query = SOMQueryObjectsRequest();
query.query.class_ = "bottle";
result:SOMQueryObjectsResponse = object_query_srv(query);
print(result);
```
The logic here is that `query.query` is of the type `SOMObject`. Any fields that are filled in within this query type will then be matched. Thus, if we fill in the `class_` attribute, we will be querying for an element of that class.

This we can also do with the category of an object using `query.query.category`

## Querying for something seen in the last minute.

```
rospy.wait_for_service('/som/objects/basic_query');
object_query_srv = rospy.ServiceProxy('/som/objects/basic_query', SOMQueryObjects);
query = SOMQueryObjectsRequest();
query.query.class_ = "bottle";
query.query.last_observed_at = rospy.Time.now() - rospy.Duration(60);
result:SOMQueryObjectsResponse = object_query_srv(query);
print(result);
```
If a temporal field is filled in, it will query for items seen in the time interval from then to now. The most likely use for this is as above where we want to seen an object of class `bottle` that was observed in the last 60 seconds. (Syntax for durations needs checking here).

## Concluding remark

Similar things can also be done for humans, observations, etc. It is also very easy to set up new collections in the database with query functionality similar to that described above.