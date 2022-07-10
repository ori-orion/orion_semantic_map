# Documentation for this system.

## Class hierachy:

 + MemoryManager                (MemoryManager.py)
    - SUMMARY:
       - This deals with the mongod connection for querying and adding things to collections.
       - It also works out what the session number is and appends it to all entries and queries if applicable. (This sometimes isn't applicable, if you're querying for a given session, or indeed if you're adding something as a prior with session number -1.)
 + TypesCollection              (CollectionManager.py)
    - SUMMARY:
       - This is a holder class for the parent type of the collection, and the adding and querying service types of relevance. If one of the service fields is `None`, then it is assumed that the respective service is unavailable.
 + ConsistencyArgs              (ObjConsistencyMapper.py)
    - SUMMARY:
       - In the drive to make this system as general as possible, there are no hard coded parameters in the core system bar stuff like `UID` and   `SESSION_NUM`. This is fine until we want to count the number of instances we've seen an object, or average over its position. `ConsistencyArgs` gives these parameter names (amoung a large number of others). If one of these fields is `None`, then it is assumed to not exist. (It should be noted that the `None` functionality has not been rigorously tested for all parameters).
 + CollectionManager            (CollectionManager.py)
    - SUMMARY:
       - This is the class that deals with the collections itself and holds the basic adding and query functions.
       - The main way of inputting parameters into this is via setting the `types:TypesCollection` parameter within `CollectionManager`. This then lets the system set up the services for querying into and adding to the collection.
    - CALLBACKS:
       - There are lists of callbacks associated with both querying and adding to the colletion. These have the dictionary for adding/querying in question, along with `metadata:dict`, for storing anything else of import (like the uid of a consistent object that's just been assigned).
    - OTHER NOTES:
    + ConsistencyChecker        (ObjConsistencyManager.py)
    + RegionManager             (RegionManager.py)
 + RelationManager              (RelationManager.py)
 + ontology_member              (Ontology.py)
 + RvizVisualisationManager     (visualisation.py)