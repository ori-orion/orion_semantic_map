## Install dependencies
pip install shapely

pip install ontospy==1.9

sudo apt install ros-kinetic-mongodb-store

## Usage
If it does not exist, create an empty folder:   semantic_mapping/db

roscore

roslaunch mongodb_store mongodb_store.launch db_path:=\`rospack find semantic_mapping\`/db

Please specify command line arguments: 

rosrun semantic_mapping som_manager.py -p <priors.csv> -o <ont.owl> -r <rois.pkl> -d <db_name (optional)>

You can try running the code with:

rosrun semantic_mapping example.py

## Available services
rospy.ServiceProxy('som/observe', SOMObserve)

rospy.ServiceProxy('som/lookup', SOMLookup)

rospy.ServiceProxy('som/delete', SOMDelete)

rospy.ServiceProxy('som/query', SOMQuery)

rospy.ServiceProxy('som/clear_database', SOMClearDatabase)

rospy.ServiceProxy('som/get_all_objects', SOMGetAllObjects)

rospy.ServiceProxy('som/check_similarity', SOMCheckSimilarity)

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

rosrun semantic_mapping som_manager.py -o '../tests/config/test_ontology.owl' -r'../tests/config/test_rooms.pkl' -p '../tests/config/priors.csv'

python -m unittest discover \`rospack find semantic_mapping\`/tests/semantic_mapping
