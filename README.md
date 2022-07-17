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
```

Note there is some logic to this. First, `{objects, observations, human_observations, humans, object_regions}` refers to different collections in the system. This tells you which collection you're adding to or querying against. Then there are the final addresses of `{input, basic_query, delete_entries, relational_query, region_query}`. These are worth going into in more detail:

### input
This is for inputting to a collection. The only input field is the SOM message you're inputting into the collection.

### basic_query
This is for performing basic queries into the system. The input is a query of the given SOM message type. You set all string/numerical/boolean fields you want to query against, and it will match any of these. Any fields that are at their default values won't be queried against. (Note that for boolean fields, this is `False` which is interesting!)

### delete_entries
This is a query of type `std_srvs.srv.Empty` and simply deletes all entries from the system.

### relational_query
This is specific to the objects query. This is for querying for instances where objects matching the query `obj1` are of the relation `relation` with respect to `obj2`. This then returns an array of matches with the responses to `obj1`, `obj2`, `relation` as well as the distance these objects are away from each other.

This was based on Marc Rigter's original SOM code.

### region_query
Regions are defined as priors within the SOM system. (They therefore have a session number of -1). They also have names. This is for querying against a given template (as per basic_query) but also within a region of name `region_name`. This will then return a list of objects within that region.

    

# Notes:

## For the addition of collections into the system in general.
The whole aim of the new system is to generalise the memory system s.t. it can be used as a memory system for any generic message. This manifests itself as being able to take an arbitrary ROS message and translate this into a dict for storage in a mongodb. There are a few conventions that need to be satisifed here.
 - Constants are conventionally all uppercase. The system currently checks only the leading character for the identification of constants. Therefore all parameters for storage in the memory system need to start with a lower case.
 - Leading underscores are how the system detects for the parameters in every python type (such as `__dict__`). Thus leading underscores are used for enumerated types within SOM messages. (If this convention is not used, enums within the message types will find their way into the query messages, thus rendering the returns empty.)
 - All entries need a `UID` entry, for referencing and querying. 
    - When you query, you only fill the parameters you are interested in. Thus, if you are querying by `UID`, you only fill out this parameter. 
 - If an object has a `SESSION_NUM` entry, this will also be filled out with the current session number. This is to allow multiple sessions to be recorded over time. 
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