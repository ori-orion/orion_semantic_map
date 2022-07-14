# Documentation for this system.

## Class hierachy:

 + `MemoryManager`                (MemoryManager.py)
    - SUMMARY:
       - This deals with the mongod connection for querying and adding things to collections.
       - It also works out what the session number is and appends it to all entries and queries if applicable. (This sometimes isn't applicable, if you're querying for a given session, or indeed if you're adding something as a prior with session number -1.)
 + `TypesCollection`              (CollectionManager.py)
    - SUMMARY:
       - This is a holder class for the parent type of the collection, and the adding and querying service types of relevance. If one of the service fields is `None`, then it is assumed that the respective service is unavailable.
 + `ConsistencyArgs`              (ObjConsistencyMapper.py)
    - SUMMARY:
       - In the drive to make this system as general as possible, there are no hard coded parameters in the core system bar stuff like `UID` and   `SESSION_NUM`. This is fine until we want to count the number of instances we've seen an object, or average over its position. `ConsistencyArgs` gives these parameter names (amoung a large number of others). If one of these fields is `None`, then it is assumed to not exist. (It should be noted that the `None` functionality has not been rigorously tested for all parameters).
 + `CollectionManager`            (CollectionManager.py)
    - SUMMARY:
       - This is the class that deals with the collections itself and holds the basic adding and query functions.
       - The main way of inputting parameters into this is via setting the `types:TypesCollection` parameter within `CollectionManager`. This then lets the system set up the services for querying into and adding to the collection.
    - CALLBACKS:
       - There are lists of callbacks associated with both querying and adding to the colletion. These have the dictionary for adding/querying in question, along with `metadata:dict`, for storing anything else of import (like the uid of a consistent object that's just been assigned).
       - Fields within `metadata`:
          - `metadata['obj_uid']`: Within consistency checking, `obj_uid` is the object uid from objects that we want to sent into the `CRSS_REF_UID` field within an observation.
          - `metadata['prevent_from_adding']`: When we are adding something, we may want to prevent it from being added (and progressing to the following callbacks). This field, once set to `True`, prevents all following callbacks from running, as well as the datapoint from being added. This can be used for suppressing double detections. The field will be created initially and set to `False`. 
    - PRIORS:
       - There are some things that we want to persist within the database, and may want to be able to query. We have reserved the session number -1 for priors. 
          - TODO:
             - Setup a prior flag to automatically set the prior information.
    - Required fields on any type stored in the system:
       - SESSION_UID:int - This is used for storing the session unique identifier. (-1 for priors).
       - UID:str - This is used for returning the unique identifier once you've added an object.
    - OTHER NOTES:
       - ENUMS:
          - These need to begin with a leading underscore so that they are ignored while querying.
    + `ConsistencyChecker`        (ObjConsistencyManager.py)
       - SUMMARY:
          - Every frame that the recognition system processes will share some objects. Object consistency checking is not done for this. This script checks for the consistency of objects across different attributes. It uses `ConsistencyArgs` as its input.
          - Note that the observations are of type `ConsistencyChecker` while the consistent objects reside in a simple `CollectionManager`. (The adding/checking for consistency is done through callbacks, so the consistent object collection can in theory be anything of type `CollectionManager`).
       - FEATURES:
          - Consistency checking.
          - Positional averaging (basic averaging, B14 Inference methods)
          - Counting number of detections.
          - Logging first and most recent observations.
          - Double detection suppression.
          - Prevention of the updating of certain attributes.
    + `RegionManager`             (RegionManager.py)
       - SUMMARY:
          - Let's say you want to query for things sitting on the table. One way of doing this would be to define a virtual box over the table. `RegionManager` defines regions (as priors), as well as querying infrastructure for it. This allows us to query for stuff within the region.
             - TODO:
                - Multi-box regions.
                - Easy setup functionality for regions.
       - ROS Message required fields on the `
 + `RelationManager`              (RelationManager.py)
    - SUMMARY:
       - It is useful to be able to say that something is above/to the left of/... something else. This implements that. The current implementation specifically references `orion_actions.msg.Relation` and uses the fields of `orion_actions.msg.Match` without specifically referencing the field.
    - ROS Message required fields on the `Relation` message:
         left:bool, right:bool, above:bool, below:bool,  behind:bool, frontof:bool, near:bool, not_near:bool, left_most:bool, right_most:bool
    - ROS Message required fields on the query return message:
         obj1:SOMObject, relation:Relation, obj2:SOMObject, distance:float64
 + `ontology_member`              (Ontology.py)
    - SUMMARY:
       - Defines the ontology tree that can be used to define the category of an element.
 + `RvizVisualisationManager`     (visualisation.py)
    - SUMMARY:
       - This defines the connection between the SOM system and rviz for detection boxes. It also scales the transparency of the box by the number of observations (up to colour_a).