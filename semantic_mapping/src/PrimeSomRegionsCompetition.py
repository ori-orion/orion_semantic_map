#!/usr/bin/env python3

from orion_actions.msg import *;
from orion_actions.srv import *;

from geometry_msgs.msg import Pose, Point;

import rospy;

def create_observation_input(name:str, position:Point):
    output = SOMAddObservationRequest();
    output.adding.class_ = name;
    output.adding.observed_at = rospy.Time.now();
    output.adding.observation_batch_num = 1;

    output.adding.obj_position.position = position;

    output.adding.size = Point(1,1,1);
    return output;

def create_map_corner_objs():
    push_to_db_srv = rospy.ServiceProxy('/som/observations/input', SOMAddObservation);
    
    positions = [
        Point(-1.036, 5.28, 0),
        Point(7.59, 6.49, 0),
        Point(9.13, -1.49, 0),
        Point(0.73, -2.99, 0)
    ];
    
    for i in range(len(positions)):
        push_to_db_srv(create_observation_input("corner_" + str(i), positions[i]));

    pass;



if __name__ == '__main__':
    rospy.init_node('prime_som_system_map_regions');

    create_map_corner_objs();