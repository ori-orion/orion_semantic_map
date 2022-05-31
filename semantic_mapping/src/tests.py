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


def test_human_observation_input():
    human_input:rospy.Service = rospy.ServiceProxy('/som/human_observations/input', orion_actions.srv.SOMAddHumanObs);
    human_query:rospy.Service = rospy.ServiceProxy('/som/humans/basic_query', orion_actions.srv.SOMQueryHumans);

    human_1:orion_actions.srv.SOMAddHumanObsRequest = orion_actions.srv.SOMAddHumanObsRequest();
    human_1.adding.task_role = "Operator";
    human_1.adding.obj_position.position.x = 1;
    human_1.adding.obj_position.position.y = 2;
    human_1.adding.obj_position.position.z = 3;
    human_1.adding.observed_at = rospy.Time.now();

    response = human_input(human_1);
    print(response);

    rospy.sleep(1);

    human_2:orion_actions.srv.SOMAddHumanObsRequest = orion_actions.srv.SOMAddHumanObsRequest();
    human_2.adding.task_role = "Operator";
    human_2.adding.obj_position.position.x = 1;
    human_2.adding.obj_position.position.y = 4;
    human_2.adding.obj_position.position.z = 3;
    human_2.adding.observed_at = rospy.Time.now();

    response = human_input(human_2);
    print(response);

    human_3:orion_actions.srv.SOMAddHumanObsRequest = orion_actions.srv.SOMAddHumanObsRequest();
    human_3.adding.task_role = "unknown";
    human_3.adding.obj_position.position.x = 1;
    human_3.adding.obj_position.position.y = 5;
    human_3.adding.obj_position.position.z = 3;
    human_3.adding.observed_at = rospy.Time.now();

    response = human_input(human_3);
    print(response);


    human_query_in:orion_actions.srv.SOMQueryHumansRequest = orion_actions.srv.SOMQueryHumansRequest();
    human_query_in.query.task_role = "Operator";
    response = human_query(human_query_in);
    print(response);

    pass;


if __name__ == '__main__':
    rospy.init_node('som_test_node');

    test_observation_input();
    test_human_observation_input();