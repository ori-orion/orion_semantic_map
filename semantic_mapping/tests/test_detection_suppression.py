#!/usr/bin/env python3
"""
Note that double detection suppression needs to be enabled for this to work.
"""

from orion_actions.msg import *;
from orion_actions.srv import *;

import rospy;

from useful_test_funcs import create_obs_instance;

def test_double_detection_suppression():
    observations_input = rospy.ServiceProxy('/som/observations/input', SOMAddObservation);
    observations_query = rospy.ServiceProxy('/som/observations/basic_query', SOMQueryObservations);
    objects_query = rospy.ServiceProxy('/som/objects/basic_query', SOMQueryObjects);

    suppressing_test_class = "suppression_test_type";

    obs_add_1 = create_obs_instance(suppressing_test_class);
    print("Suppress return 1:", observations_input(obs_add_1));
    obs_add_2 = create_obs_instance(suppressing_test_class, x = 0.001);
    print("Suppress return 1:", observations_input(obs_add_2));

    obs_query = SOMQueryObservationsRequest();
    obs_query.query.class_ = suppressing_test_class;
    obs_query_response:SOMQueryObservationsResponse = observations_query(obs_query);
    assert(len(obs_query_response.returns) == 1);
    print("Assertion on number of observations has passed.")

    obj_query = SOMQueryObjectsRequest();
    obj_query.query.class_ = suppressing_test_class;
    obj_query_response:SOMQueryObjectsResponse = objects_query(obj_query);
    assert(len(obj_query_response.returns) == 1);
    print("Assertion on number of objects has passed.")

if __name__ == '__main__':
    rospy.init_node('testing_double_detection_suppression')
    test_double_detection_suppression();