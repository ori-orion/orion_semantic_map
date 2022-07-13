#!/usr/bin/env python3

from orion_actions.msg import *;
from orion_actions.srv import *;

from geometry_msgs.msg import Pose, Point;

import rospy;

import math;

import std_srvs.srv

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

def create_region():
    rospy.wait_for_service('/som/object_regions/delete_entries');
    rospy.wait_for_service('/som/object_regions/input');
    region_delete_srv = rospy.ServiceProxy('/som/object_regions/delete_entries', std_srvs.srv.Empty);
    region_delete_srv(std_srvs.srv.EmptyRequest());
    region_input = rospy.ServiceProxy('/som/object_regions/input', SOMAddRegion);

    region_adding = SOMAddRegionRequest();
    region_adding.adding.dimension.x = 4.5;
    region_adding.adding.dimension.y = 5.5;
    region_adding.adding.dimension.z = 2;

    region_adding.adding.corner_loc.translation.x = 0.73
    region_adding.adding.corner_loc.translation.y = -2.99
    region_adding.adding.corner_loc.translation.z = 0

    angle = 5;
    region_adding.adding.corner_loc.rotation.x = 0;
    region_adding.adding.corner_loc.rotation.y = 0;
    region_adding.adding.corner_loc.rotation.w = math.cos(angle * math.pi/180);
    region_adding.adding.corner_loc.rotation.z = math.sin(angle * math.pi/180);

    region_adding.adding.name = "arena_boundry";

    region_input(region_adding);



if __name__ == '__main__':
    rospy.init_node('prime_som_system_map_regions');

    create_region();

    # create_map_corner_objs();