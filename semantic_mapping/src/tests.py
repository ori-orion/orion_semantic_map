#!/usr/bin/env python3

import rospy

import geometry_msgs.msg;

import orion_actions.srv
import orion_actions.msg


def create_obs_instance(class_, x, y, z) -> orion_actions.srv.SOMAddObservationRequest:
    output = orion_actions.srv.SOMAddObservationRequest();
    output.adding.class_ = class_;
    output.adding.obj_position.position.x = x;
    output.adding.obj_position.position.y = y;
    output.adding.obj_position.position.z = z;
    output.adding.observed_at = rospy.Time.now();
    
    return output;



def test_observation_input():
    push_to_db_srv = rospy.ServiceProxy('/som/observations/input', orion_actions.srv.SOMAddObservation);

    adding = create_obs_instance("bottle", 0.1, 0, 0);    
    adding.adding.category = "vessel";
    
    obj_return = push_to_db_srv(adding);
    print(obj_return);

    rospy.sleep(2);

    get_obj_from_db_srv = rospy.ServiceProxy('/som/objects/basic_query', orion_actions.srv.SOMQueryObjects);

    querying = orion_actions.srv.SOMQueryObjectsRequest();
    querying.query.class_ = "bottle";
    
    query_return = get_obj_from_db_srv(querying);
    print(query_return);

    adding = create_obs_instance("bottle", -0.05, 0, 0);
    adding.adding.category = "vessel";
    
    obj_return = push_to_db_srv(adding);
    print(obj_return);

    rospy.sleep(2);

    adding = create_obs_instance("bottle", 1,0,0);
    adding.adding.category = "vessel";
    
    obj_return = push_to_db_srv(adding);
    print(obj_return);

    rospy.sleep(2);

    adding = create_obs_instance("apple", -0.1,0,0);
    adding.adding.category = "fruit";
    
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


def test_obj_relational_query():
    # NOTE: the suffix of `_rel` is to distinguish objects added here from
    # those added in the other tests. 
    push_to_db_srv = rospy.ServiceProxy('/som/observations/input', orion_actions.srv.SOMAddObservation);
    relational_query_srv = rospy.ServiceProxy('/som/objects/relational_query', orion_actions.srv.SOMRelObjQuery);
    
    obj1 = create_obs_instance("window_rel", 1, 0,0.2);
    push_to_db_srv(obj1);

    obj2 = create_obs_instance("banana_rel", 1.5, 0.2, 0);
    push_to_db_srv(obj2);

    query1 = orion_actions.srv.SOMRelObjQueryRequest();
    query1.obj1.class_ = "window_rel";
    query1.obj2.class_ = "banana_rel";
    query1.current_robot_pose = geometry_msgs.msg.Pose();
    query1_output:orion_actions.srv.SOMRelObjQueryResponse = relational_query_srv(query1);
    print(query1_output);

    # Note that here the window is infront of the banana. Also, the banana is to the right of the window 
    # (Construct a set of cartesian coordinates to show this.)
    # There should also be only one output (given that there are only these two objects in position at the
    # moment.)
    assert(len(query1_output.matches) == 1);
    match0:orion_actions.msg.Match = query1_output.matches[0];
    assert(match0.relation.near == True);
    assert(match0.relation.right == True);
    assert(match0.relation.left == False);
    assert(match0.relation.frontof == True);
    assert(match0.relation.behind == False);
    assert(match0.relation.above == True);
    assert(match0.relation.below == False);

    print("Behind query");
    query2 = orion_actions.srv.SOMRelObjQueryRequest();
    query2.obj1.class_ = "window_rel";
    query2.relation.behind = True;
    query2_output:orion_actions.srv.SOMRelObjQueryResponse = relational_query_srv(query2);
    print(query2_output);
    for element in query2_output.matches:
        element:orion_actions.msg.Match;
        assert(element.relation.behind == True);

    pass;


if __name__ == '__main__':
    rospy.init_node('som_test_node');

    test_observation_input();
    test_human_observation_input();
    test_obj_relational_query();