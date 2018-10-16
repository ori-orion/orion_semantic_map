# Object Database Manager	

## Launch
`roslaunch object_database_manager object_database_manager_node.launch`

For now all the waypoints that are being used need to be in object_database_manager/waypoints.yaml. 
Reading the waypoints from the /topological_map topic had a bug that I haven't been able to fix yet. 

## Services                                                                                                                                                                                                                                                                 
* `/object_query_server`
	* input: name of the object as a string
	* output:
		* probabilites[]
		* waypoints[]

### Please give feedback on stupid code or unknowingly ignoring ROS conventions.
