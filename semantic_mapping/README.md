## Usage
If it does not exist, create an empty folder:   semantic_mapping/db

roscore

roslaunch mongodb_store mongodb_store.launch db_path:=\`rospack find semantic_mapping\`/db

rosrun semantic_mapping som_manager.py <ontology.owl> <som_rois.pkl> 

You can try running the code with:

rosrun semantic_mapping example.py

## Available services
rospy.ServiceProxy('som/observe', SOMObserve)
rospy.ServiceProxy('som/lookup', SOMLookup)
rospy.ServiceProxy('som/delete', SOMDelete)
rospy.ServiceProxy('som/query', SOMQuery)
rospy.ServiceProxy('som/clear_database', SOMClearDatabase)
rospy.ServiceProxy('som/get_all_objects', SOMGetAllObjects)

#### Visualise
rosrun rviz rviz

Display:

/som/obj_vis/update

/som/roi_vis

Clicking on objects in Rviz prints out the information stored about that object.


## Creating ROIs
To edit the SOM ROIs:

roscore

roslaunch mongodb_store mongodb_store.launch db_path:=\`rospack find semantic_mapping\`/db

rosrun semantic_mapping roi_manager.py

Its best to make the rooms overlap, as an object in a gap between rooms will be assigned the room_name "NotInRoom". An object in multiple overlapping rooms will be assigned to one of the rooms. 

#### Save ROIS
Right click on one of the markers, and click "Save ROIs" which saves the ROI objects to a pickle file.

## Run tests
roscore

roslaunch mongodb_store mongodb_store.launch db_path:=\`rospack find semantic_mapping\`/db

rosrun semantic_mapping som_manager.py '../tests/config/test_ontology.owl' '../tests/config/test_rooms.pkl'

python -m unittest discover \`rospack find semantic_mapping\`/tests/semantic_mapping
