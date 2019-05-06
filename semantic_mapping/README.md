## Usage
If it does not exist, create an empty folder:   semantic_mapping/db

roscore

roslaunch mongodb_store mongodb_store.launch db_path:=\`rospack find semantic_mapping\`/db

rosrun semantic_mapping som_manager.py <ontology.owl> <som_rois.pkl> <clear_dbs_bool>

You can try running the code with:

rosrun semantic_mapping test_database.py

#### Visualise
rosrun rviz rviz

Display:
/som/obj_vis/update
/som/rois

Clicking on objects in Rviz prints out the information stored about that object.


## Creating ROIs
To edit the SOM ROIs:

roscore

roslaunch mongodb_store mongodb_store.launch db_path:=\`rospack find semantic_mapping\`/db

rosrun semantic_mapping roi_manager.py

#### Save ROIS
Right click on one of the markers, and click "Save ROIs" which saves the ROI objects to a pickle file.

## Run tests
roscore
roslaunch mongodb_store mongodb_store.launch db_path:=\`rospack find semantic_mapping\`/db
rosrun semantic_mapping som_manager.py '../tests/config/test_ontology.owl' '../tests/config/test_rooms.pkl'
python -m unittest discover \`rospack find semantic_mapping\`/tests/semantic_mapping
