#!/usr/bin/env python3

import rospy

import orion_actions.srv
import orion_actions.msg

def test_observation_input():
    push_to_db_srv = rospy.ServiceProxy('/som/observations/input', orion_actions.srv.SOMAddObservation);

    adding = orion_actions.srv.SOMAddObservationRequest();
    adding.adding.class_ = "bottle";
    adding.adding.category = "vessel";
    adding.adding.obj_position.position.x = 0.1;
    adding.adding.obj_position.position.y = 0;
    adding.adding.obj_position.position.z = 0;
    adding.adding.observed_at = rospy.Time.now();

    obj_return = push_to_db_srv(adding);
    print(obj_return);

    rospy.sleep(2);

    get_obj_from_db_srv = rospy.ServiceProxy('/som/objects/basic_query', orion_actions.srv.SOMQueryObjects);

    querying = orion_actions.srv.SOMQueryObjectsRequest();
    querying.query.class_ = "bottle";
    
    query_return = get_obj_from_db_srv(querying);
    print(query_return);

    adding = orion_actions.srv.SOMAddObservationRequest();
    adding.adding.class_ = "bottle";
    adding.adding.category = "vessel";
    adding.adding.obj_position.position.x = -0.05;
    adding.adding.obj_position.position.y = 0;
    adding.adding.obj_position.position.z = 0;
    adding.adding.observed_at = rospy.Time.now();

    obj_return = push_to_db_srv(adding);
    print(obj_return);

    rospy.sleep(2);

    adding = orion_actions.srv.SOMAddObservationRequest();
    adding.adding.class_ = "bottle";
    adding.adding.category = "vessel";
    adding.adding.obj_position.position.x = 1;
    adding.adding.obj_position.position.y = 0;
    adding.adding.obj_position.position.z = 0;
    adding.adding.observed_at = rospy.Time.now();

    obj_return = push_to_db_srv(adding);
    print(obj_return);

    rospy.sleep(2);

    adding = orion_actions.srv.SOMAddObservationRequest();
    adding.adding.class_ = "apple";
    adding.adding.category = "fruit";
    adding.adding.obj_position.position.x = -0.1;
    adding.adding.obj_position.position.y = 0;
    adding.adding.obj_position.position.z = 0;
    adding.adding.observed_at = rospy.Time.now();

    obj_return = push_to_db_srv(adding);
    print(obj_return);
    

    # get_obvs_from_db_srv = rospy.ServiceProxy('/som/observations/basic_query', orion_actions.srv.SOMQueryObservations);

    # querying = orion_actions.srv.SOMQueryObservationsRequest();
    # querying.query.category = "vessel";
    
    # query_return = get_obvs_from_db_srv(querying);
    # print(query_return);


    get_obj_from_db_srv = rospy.ServiceProxy('/som/objects/basic_query', orion_actions.srv.SOMQueryObjects);

    querying = orion_actions.srv.SOMQueryObjectsRequest();
    querying.query.class_ = "bottle";
    
    query_return = get_obj_from_db_srv(querying);
    print(query_return);



    pass;

if __name__ == '__main__':
    rospy.init_node('som_test_node');

    test_observation_input();