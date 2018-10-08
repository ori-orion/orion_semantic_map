# Object Location Service	

## Launch
`roslaunch object_location_service object_location_service_server.launch`

## Services                                                                                                                                                                                                                                                                 
* `/object_query_server`
	* input: name of the object as a string
	* output: if it exists in the database you will get the probability distribution back
* `/object_insertion_server`
	* input: 
		* name of the object as a string
		* waypoint(numeral) the object is at as a string
	* output: nothing

## ToDo
* implement the version where the objects + waypoints are being published continually on a topic
* how to deal with objects being seen at several waypoints (maybe due to error?)

### Please give feedback on stupid code or unknowingly ignoring ROS conventions.
