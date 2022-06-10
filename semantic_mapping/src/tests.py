#!/usr/bin/env python3

import rospy

import geometry_msgs.msg;

import orion_actions.srv
import orion_actions.msg


def create_obs_instance(class_, x, y, z, batch_num=0, category="") -> orion_actions.srv.SOMAddObservationRequest:
    output = orion_actions.srv.SOMAddObservationRequest();
    output.adding.class_ = class_;
    output.adding.obj_position.position.x = x;
    output.adding.obj_position.position.y = y;
    output.adding.obj_position.position.z = z;
    output.adding.observation_batch_num = batch_num;
    output.adding.category = category;
    output.adding.observed_at = rospy.Time.now();
    
    return output;



def test_observation_input():
    push_to_db_srv = rospy.ServiceProxy('/som/observations/input', orion_actions.srv.SOMAddObservation);
    get_obj_from_db_srv = rospy.ServiceProxy('/som/objects/basic_query', orion_actions.srv.SOMQueryObjects);

    adding = create_obs_instance("bottle", 0.1, 0, 0, batch_num=0, category="vessel");
    obj_return = push_to_db_srv(adding);
    adding = create_obs_instance("bottle", 0.25,0,0, batch_num=0, category="vessel");
    obj_return = push_to_db_srv(adding);

    adding = create_obs_instance("person", 0,2,0, batch_num=0);
    obj_return = push_to_db_srv(adding);

    rospy.sleep(2);

    # Note that this is within the 1m distance for a person but further away than the 1m distance
    # given for persons but greater than the 0.3m distance given for other objects.
    adding = create_obs_instance("person", 0,2.7,0, batch_num=1);
    obj_return = push_to_db_srv(adding);
    adding = create_obs_instance("person", 0,3,0, batch_num=1);
    obj_return = push_to_db_srv(adding);

    adding = create_obs_instance("bottle", -0.05, 0, 0, batch_num=1, category="vessel");
    obj_return = push_to_db_srv(adding);
    print(obj_return);
    
    print("first query for bottle. Expecting 2 returns.")
    querying = orion_actions.srv.SOMQueryObjectsRequest();
    querying.query.class_ = "bottle";
    query_return:orion_actions.srv.SOMQueryObjectsResponse = get_obj_from_db_srv(querying);
    assert(len(query_return.returns) == 2);

    print("first query for person. Expecting 2 returns.")
    querying = orion_actions.srv.SOMQueryObjectsRequest();
    querying.query.class_ = "person";
    query_return:orion_actions.srv.SOMQueryObjectsResponse = get_obj_from_db_srv(querying);
    print(query_return.returns);
    assert(len(query_return.returns) == 2);

    rospy.sleep(2);

    adding = create_obs_instance("apple", -0.1,0,0);
    adding.adding.category = "fruit";
    
    obj_return = push_to_db_srv(adding);
    print(obj_return);

    rospy.sleep(2);
    adding = create_obs_instance("person", 0,0,0);
    adding.adding.category = "person";
    obj_return = push_to_db_srv(adding);
    print(obj_return);

    rospy.sleep(2);
    adding = create_obs_instance("person", 1,1,0);
    adding.adding.category = "person";
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

    print("\tOperator query...");
    human_query_in:orion_actions.srv.SOMQueryHumansRequest = orion_actions.srv.SOMQueryHumansRequest();
    human_query_in.query.task_role = "Operator";
    response = human_query(human_query_in);
    print(response);

    print("\tEmpty query...");
    human_empty_query:orion_actions.srv.SOMQueryHumansRequest = orion_actions.srv.SOMQueryHumansRequest();
    response = human_query(human_empty_query);
    print(response);

    pass;


def test_obj_relational_query():
    # NOTE: the suffix of `_rel` is to distinguish objects added here from
    # those added in the other tests. 
    push_to_db_srv = rospy.ServiceProxy('/som/observations/input', orion_actions.srv.SOMAddObservation);
    relational_query_srv = rospy.ServiceProxy('/som/objects/relational_query', orion_actions.srv.SOMRelObjQuery);
    
    obj1 = create_obs_instance("window_rel", 1, 0,0.2);
    obj1.adding.category = "a";
    push_to_db_srv(obj1);

    obj2 = create_obs_instance("banana_rel", 1.5, 0.2, 0);
    obj2.adding.category = "b";
    push_to_db_srv(obj2);

    obj1 = create_obs_instance("floor_rel", 1, 0,-0.2);
    obj1.adding.category = "a";
    push_to_db_srv(obj1);

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

     
    query3 = orion_actions.srv.SOMRelObjQueryRequest();
    query3.obj1.category = "a";
    query3.obj2.category = "b";
    query3.relation.above = True;
    query3.current_robot_pose = geometry_msgs.msg.Pose();
    query3_output:orion_actions.srv.SOMRelObjQueryResponse = relational_query_srv(query3);
    assert(len(query3_output.matches) == 1);
    match0:orion_actions.msg.Match = query3_output.matches[0];
    assert(match0.obj1.class_ == "window_rel");
    assert(match0.obj2.class_ == "banana_rel");


    pass;


def uid_input_test():
    push_to_db_srv = rospy.ServiceProxy('/som/observations/input', orion_actions.srv.SOMAddObservation);
    get_obj_from_db_srv = rospy.ServiceProxy('/som/objects/basic_query', orion_actions.srv.SOMQueryObjects);

    adding = create_obs_instance("uid_test_input_obj", 0.1, 0, 0, batch_num=0);
    obj_return:orion_actions.srv.SOMAddObservationResponse = push_to_db_srv(adding);

    print("Checking UID queries work.")
    querying = orion_actions.srv.SOMQueryObjectsRequest();
    querying.query.UID = obj_return.UID;
    query_return:orion_actions.srv.SOMQueryObjectsResponse = get_obj_from_db_srv(querying);
    # print(query_return.returns);
    assert(len(query_return.returns) == 1);

    print("Checking SESSION_NUM queries work and that they are ordered by latest batch number")
    querying = orion_actions.srv.SOMQueryObjectsRequest();
    querying.query.SESSION_NUM = 1; # The first session.
    query_return:orion_actions.srv.SOMQueryObjectsResponse = get_obj_from_db_srv(querying);
    # print(query_return.returns);
    assert(len(query_return.returns) != 0);
    return_batch_nums = [];
    for element in query_return.returns:
        element:orion_actions.msg.SOMObject;
        return_batch_nums.append(element.last_observation_batch);
        l = len(return_batch_nums);
        if (l >= 2):
            assert(return_batch_nums[l-2] >= return_batch_nums[l-1]);
    print(return_batch_nums);

def test_category_callback():
    push_to_db_srv = rospy.ServiceProxy('/som/observations/input', orion_actions.srv.SOMAddObservation);
    get_obj_from_db_srv = rospy.ServiceProxy('/som/objects/basic_query', orion_actions.srv.SOMQueryObjects);

    adding = create_obs_instance("food_tray", 0,0,1);
    push_to_db_srv(adding);

    print("Checking category assignment");
    querying = orion_actions.srv.SOMQueryObjectsRequest();
    querying.query.class_ = "food_tray";
    query_return:orion_actions.srv.SOMQueryObjectsResponse = get_obj_from_db_srv(querying);
    assert(len(query_return.returns) != 0);
    return_0:orion_actions.msg.SOMObject = query_return.returns[0];
    assert(return_0.category == "containers");


if __name__ == '__main__':
    rospy.init_node('som_test_node');

    test_observation_input();
    test_human_observation_input();
    test_obj_relational_query();
    uid_input_test();
    test_category_callback();

    t = rospy.Time.now();
    print(t.secs);
    print(t.nsecs);
