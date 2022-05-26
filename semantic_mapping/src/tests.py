#!/usr/bin/env python3

import rospy

import orion_actions.srv
import orion_actions.msg

def test_observation_input():
    push_to_db_srv = rospy.ServiceProxy('/som/observations/input', orion_actions.srv.SOMAddObservation);

    adding = orion_actions.srv.SOMAddObservationRequest();
    adding.adding.class_ = "bottle";
    adding.adding.category = "vessel";

    obj_return = push_to_db_srv(adding);
    print(obj_return);

    adding = orion_actions.srv.SOMAddObservationRequest();
    adding.adding.class_ = "bottle_2";
    adding.adding.category = "vessel";

    obj_return = push_to_db_srv(adding);
    print(obj_return);
    

    get_from_db_srv = rospy.ServiceProxy('/som/observations/basic_query', orion_actions.srv.SOMGetObservations);

    querying = orion_actions.srv.SOMGetObservationsRequest();
    querying.query.category = "vessel";
    
    query_return = get_from_db_srv(querying);
    print(query_return);
    print(len(query_return));

    pass;

if __name__ == '__main__':
    rospy.init_node('som_test_node');

    test_observation_input();