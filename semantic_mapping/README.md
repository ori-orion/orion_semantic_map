### Usage
If it does not exist, create an empty folder:   semantic_mapping/db

roscore 

roslaunch mongodb_store mongodb_store.launch db_path:='rospack find semantic_mapping'/db 

rosrun semantic_mapping som_manager.py  


You can try running the code with:

rosrun semantic_mapping test_database.py 
