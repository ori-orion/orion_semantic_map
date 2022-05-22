# orion_semantic_map
ORIon Semantic Mapping.

READMEs are included in each subfolder.

Current active branch is `master`.

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
   - Maybe some sort of callback function within the CollectionManager is in order.