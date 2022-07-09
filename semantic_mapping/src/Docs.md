# Documentation for this system.

## Class hierachy:

 + MemoryManager                (MemoryManager.py)
    - SUMMARY:
        Deals with the mongod connection for querying and adding things to collections.
        It also deals with the session number.  
 + TypesCollection              (CollectionManager.py)
 + ConsistencyArgs              (ObjConsistencyMapper.py)
 + CollectionManager            (CollectionManager.py)
    + ConsistencyChecker        (ObjConsistencyManager.py)
    + RegionManager             (RegionManager.py)
 + RelationManager              (RelationManager.py)
 + ontology_member              (Ontology.py)
 + RvizVisualisationManager     (visualisation.py)