# Did we observe the object in this session or a different one?
int16 session_num

string uid

# The class/type of an object. (i.e., table, apple, potted plant).
string class

# Apples are fruit are object. This will give one level up (so apple -> fruit).
string category

geometry_msgs/Point obj_position

bool pickupable

# Give the pose information necessary for full identification.
# (For testing purposes, if tf can't be found, this will be set to NAN).
geometry_msgs/Pose base_pose
geometry_msgs/Pose head_pose

# When did the observation happen?
timestamp observed_at

# Some indication of the object size. (This will be approximate to begin with).
geometry_msgs/size size

# Do we expect the object to move?
bool static


