# SOM system overhaul.

## Main ideas.
 - Back compatibility with the previous system (sort of). 
    - This is to make sure it is easy to integrate into the existing systems that use it. (Note that not many systems actually properly use it).- This essentially means maintaining...
        - som/lookup
        - som/query
 - Direct reading of the observation data.
    - I want to get rid of the detections_to_observations.py script completely. It doesn't make sense to have it.
 - Some sort of measure of expandability.
    - For the EBBHRD project, I managed to create scripts to convert python message/service types from and to dictionary types without pre-knowledge of their contents. This makes me think it should be possible to load in arbitrary types if we felt so inclined. This would then simplify the overall system, as well as leading to an easy route to adding some of the EBB functionality directly into the memory system which might be useful for explainability. 
        - MLE/MAP would then be a modifier that acts on a collection (with pre-knowledge of the properties to conduct MLE/MAP over)
            - There would be options about which system to use (which would probably need to include a low pass filter).
            - The MLE/MAP system would then act on a given "observation" collection to create an "observation" collection.
        - This could also be used to load in point clouds if so desired. 
        - Another benefit is that all you would need to do to expand/contract a given entry would be to add/take away the given entry from the .msg/.srv definition and the change would carry right the way through. 
 - I'd quite like to have covariance matrices per observation.
    - We know that the depth is a lot less certain than left/right and up/down, so we might as well use that if we can. 
    - This is better handled internally.

## Proposed overall structure:

 - MemoryManager
    - For dealing with mongodb. (I'm currently tempted to move away from mongodb_store, though I'm not sure if it's being used elsewhere/what other properties make it desirable).
        - We'd probably also still end up running mongodb_store becuase encasing the mongodb server in a ROS node is just plain useful.
    - I'd also quite like to end up with a session number so that we can look back at past runs and actually have the data/know one run from the next because that feels useful. (This would involve a session counter which is simple to implement).
 - CollectionManager
    - Deals with the addition of arbitrary message types.
    - Sub classes:
        - ObjConsistencyTracker
            - Deals with the implementation of MLE/MAP/Low pass filters for properties that are given in an argument list.
            - Services:
                - tf publisher
                    - To make it back compatible with the current manipulation stack. (Not sure if this is necessary!).
    - Services:
        - query
            - The point here is that we may well want to look up an object merely based on qualities we know it has (such as type or colour). 
            - This may seem like a double for the RelationChecker but 
                - The RelationChecker code may well end up partially being based on this.
                - The RelationChecker will only be operating on one of the collections.
        - lookup
            - Looks an object up by its UID.
            - The idea is that the UID becomes a lot more important overall.
 - RelationChecker
    - Deals with the service call for getting object relations. (Will contain the ObjConsistencyTracker object for generic objects.)
    - Likely will (almost) directly implement Mark's code.    

## Things I'm unsure of.
 - So I'm not sure how much code to re-use/whether to simply rewrite it. 
 - There is some functionality Re ontologies, roi stuff and visualisations, some of which would definitely be good to replicate, some of which I'm unsure about, but it might be a plan to work out the functionality of some of it.
    - Ontology -> speech recognition???
 - Importance of priors.
    - Could these be session -1?
    - Are they properly going to be used anyway?
    - Not sure.


Notes for stuff we want. 
 - Priors.
    - We know that there is a Mug on the shelf.
    - We know that there is a shelf in the system. 
 - Add something to check if an object is currently in sight.
 - Am I near the surface we want to be near to pick up the object?
 - Localisation => Low pass filter.
 - Assume localisation is good enough. 
 - Bounding box
    - Assume symmetrically 
 - Things to keep track of (Obj Database)
    - UID
    - Class of object (rather than type)
        - Match humans across databases. 
    - HumanRef
        - Ref to human entry if applicable.
    - Category of object
    - Colour
        - Last update wins OR priors
    - Global Position
    - Pickupable
    - Last observed time stamp
    - First observed time stamp
        - ROS time
    - Robot pose at last observation
        - Base pose
        - Head camera pose
    - Size
        - Assume rotationally symmetric???
        - Octagonal???
        - Take the width and extrude 
            - by width (assuming its width is the same as the depth)?
            - by a constant (sensible) amount
        - The segmentation algorithm takes an approximate bounding box to start the segmentation. Having this would be EXTREMELY useful. 
        - The other option: We have a set of sizes
            - Smallest width = smallest width
            - Largest width = Largest width combined with smallest width with trig.
            - This is also possible???
        - FINAL notes.
        - Stuff we know for certain:
            - Width and Height   
    - static
        - Do we expect the object to move?
        - vel = 0.
    - dont_update    
        - Some objects we don't expect to ever move (shelf).      
    - DON'T track orientation
 - Things to keep track of (Human Database)
    - ObjRef
        - Reference the object database
    - name 
    - age
    - gender
    - ...
 - Ontology???

Recognition stuff.
 - Median filter for recognition??? 

Objects collection
Humans collection