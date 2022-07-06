# orion_semantic_map
ORIon Semantic Mapping.

READMEs are included in each subfolder.

Current active branch is `noetic`.

# Notes on current implementation...

## ROS message definitions:

    - /som/human_observations
        - For the adding of observations of humans.
        - /input
    - /som/humans
        - For the storing of human entities.
        - /basic_query
    - /som/observations
        - For the adding of observations of objects.
        - /input
        - /basic_query
    - /som/objects
        - For storing the object entities.
        - /basic_query
        - /input
            - To mainly be used for updating a given entry (done by setting UID).
    - /som/object_regions
        - For storing (and querying) regions.
        - /input
        - /basic_query
        - /region_query
    

# Notes:

## For the addition of collections into the system in general.
The whole aim of the new system is to generalise the memory system s.t. it can be used as a memory system for any generic message. This manifests itself as being able to take an arbitrary ROS message and translate this into a dict for storage in a mongodb. There are a few conventions that need to be satisifed here.
 - Constants are conventionally all uppercase. The system currently checks only the leading character for the identification of constants. Therefore all parameters for storage in the memory system need to start with a lower case.
 - Leading underscores are how the system detects for the parameters in every python type (such as `__dict__`). Thus leading underscores are also out.
 - All entries need a `uid` entry, for referencing and querying. 
    - When you query, you only fill the parameters you are interested in. Thus, if you are querying by `uid`, you only fill out this parameter. 
 - If an object has a `session_num` entry, this will also be filled out with the current session number. This is to allow multiple sessions to be recorded over time. 
    - Priors will be stored with `session_num=-1`.

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



Keytravel
   - Fly out on the Saturday (16th)
