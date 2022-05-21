#!/usr/bin/env python3

import rospy

import orion_actions.srv
import orion_actions.msg

def test_observation_input():
    push_to_db_srv = rospy.ServiceProxy('/som/observations/input', orion_actions.srv.SOMAddObservation);#

    adding = orion_actions.srv.SOMAddObservationRequest();
    adding.adding.type = "bottle";
    adding.adding.category = "vessel";

    obj_return = push_to_db_srv(adding);
    print(obj_return);
    # print(obj_return.)

    pass;

if __name__ == '__main__':
    rospy.init_node('som_test_node');

    test_observation_input();