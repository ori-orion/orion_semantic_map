#!/usr/bin/env python3

import rospy

import geometry_msgs.msg;

import orion_actions.srv
import orion_actions.msg

import std_srvs.srv;


def create_obs_instance(class_, x=0, y=0, z=0, batch_num=0, category="") -> orion_actions.srv.SOMAddObservationRequest:
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

    rospy.sleep(0.2);

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
    print(query_return);

    print("first query for person. Expecting 2 returns.")
    querying = orion_actions.srv.SOMQueryObjectsRequest();
    querying.query.class_ = "person";
    query_return:orion_actions.srv.SOMQueryObjectsResponse = get_obj_from_db_srv(querying);
    # print(query_return.returns);
    assert(len(query_return.returns) == 2);

    rospy.sleep(0.2);

    adding = create_obs_instance("apple", -0.1,0,0);
    adding.adding.category = "fruit";
    obj_return = push_to_db_srv(adding);
    print(obj_return);

    rospy.sleep(0.2);
    adding = create_obs_instance("person", 0,0,0);
    adding.adding.category = "person";
    obj_return = push_to_db_srv(adding);
    print(obj_return);

    rospy.sleep(0.2);
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
    # print(query_return);
    assert(len(query_return.returns) != 0)
    print("Length of returns for `bottle`:", len(query_return.returns));

    pass;


def test_human_observation_input():
    human_input:rospy.Service = rospy.ServiceProxy('/som/human_observations/input', orion_actions.srv.SOMAddHumanObs);
    human_query:rospy.Service = rospy.ServiceProxy('/som/humans/basic_query', orion_actions.srv.SOMQueryHumans);

    human_1:orion_actions.srv.SOMAddHumanObsRequest = orion_actions.srv.SOMAddHumanObsRequest();
    human_1.adding.task_role = "Operator";
    human_1.adding.object_uid = "test_uid_1";
    human_1.adding.obj_position.position.x = 1;
    human_1.adding.obj_position.position.y = 2;
    human_1.adding.obj_position.position.z = 3;
    human_1.adding.observed_at = rospy.Time.now();

    response = human_input(human_1);
    print(response);

    rospy.sleep(0.1);

    human_2:orion_actions.srv.SOMAddHumanObsRequest = orion_actions.srv.SOMAddHumanObsRequest();
    human_2.adding.task_role = "Operator";
    human_2.adding.object_uid = "test_uid_2";
    human_2.adding.obj_position.position.x = 1;
    human_2.adding.obj_position.position.y = 4;
    human_2.adding.obj_position.position.z = 3;
    human_2.adding.observed_at = rospy.Time.now();

    response = human_input(human_2);
    print(response);

    human_3:orion_actions.srv.SOMAddHumanObsRequest = orion_actions.srv.SOMAddHumanObsRequest();
    human_3.adding.task_role = "unknown";
    human_3.adding.object_uid = "test_uid_3";
    human_3.adding.obj_position.position.x = 1;
    human_3.adding.obj_position.position.y = 5;
    human_3.adding.obj_position.position.z = 3;
    human_3.adding.observed_at = rospy.Time.now();

    response = human_input(human_3);
    print(response);

    print("\tOperator query...");
    human_query_in:orion_actions.srv.SOMQueryHumansRequest = orion_actions.srv.SOMQueryHumansRequest();
    human_query_in.query.task_role = "Operator";
    response:orion_actions.srv.SOMQueryHumansResponse = human_query(human_query_in);
    assert(len(response.returns) != 0);
    # print(response);

    print("\tEmpty query...");
    human_empty_query:orion_actions.srv.SOMQueryHumansRequest = orion_actions.srv.SOMQueryHumansRequest();
    response:orion_actions.srv.SOMQueryHumansResponse = human_query(human_empty_query);
    assert(len(response.returns) != 0);
    # print(response);


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
    print("Length of relational query 1: ", len(query1_output.matches));
    # print(query1_output);

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
    print("Length of relational query 2: ", len(query2_output.matches));
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


def uid_input_test():
    push_to_db_srv = rospy.ServiceProxy('/som/observations/input', orion_actions.srv.SOMAddObservation);
    get_obj_from_db_srv = rospy.ServiceProxy('/som/objects/basic_query', orion_actions.srv.SOMQueryObjects);

    adding = create_obs_instance("uid_test_input_obj", 0.1, 0, 0, batch_num=0);
    obj_return:orion_actions.srv.SOMAddObservationResponse = push_to_db_srv(adding);

    print("Checking UID queries work.")
    querying = orion_actions.srv.SOMQueryObjectsRequest();
    querying.query.HEADER.UID = obj_return.UID;
    query_return:orion_actions.srv.SOMQueryObjectsResponse = get_obj_from_db_srv(querying);
    # print(query_return.returns);
    assert(len(query_return.returns) == 1);

    print("Checking SESSION_NUM queries work and that they are ordered by latest batch number")
    querying = orion_actions.srv.SOMQueryObjectsRequest();
    querying.query.HEADER.SESSION_NUM = 1; # The first session.
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


def test_covariance_method():
    print("Test covariance method");
    push_to_db_srv = rospy.ServiceProxy('/som/observations/input', orion_actions.srv.SOMAddObservation);
    get_obj_from_db_srv = rospy.ServiceProxy('/som/objects/basic_query', orion_actions.srv.SOMQueryObjects);

    obj_name = "covariance_test";
    adding_1 = create_obs_instance(obj_name, 0.1,0,0, 2);
    adding_1.adding.covariance_mat = [1,0,0,0,10,0,0,0,1];
    print("\tadding_obj_1");
    push_to_db_srv(adding_1);
    adding_2 = create_obs_instance(obj_name, 0,0.1,0, 3);
    adding_2.adding.covariance_mat = [10,0,0,0,1,0,0,0,1];
    print("\tadding_obj_2")
    push_to_db_srv(adding_2);

    query = orion_actions.srv.SOMQueryObjectsRequest();
    query.query.class_ = obj_name;
    print("\tQuerying for the results");
    query_response:orion_actions.srv.SOMQueryObjectsResponse = get_obj_from_db_srv(query);
    print(len(query_response.returns));


def test_updating_entry():
    print("Testing the updating of an entry");

    push_to_db_srv = rospy.ServiceProxy('/som/observations/input', orion_actions.srv.SOMAddObservation);
    get_obs_from_db_srv = rospy.ServiceProxy('/som/observations/basic_query', orion_actions.srv.SOMQueryObservations);

    adding = create_obs_instance("update_entry_test", 0.1, 0, 0, batch_num=0, category="unupdated_category");
    obj_return:orion_actions.srv.SOMAddObservationResponse = push_to_db_srv(adding);

    querying = orion_actions.srv.SOMQueryObservationsRequest();
    querying.query.class_ = "update_entry_test";
    query_response = get_obs_from_db_srv(querying);
    assert(len(query_response.returns) == 1);
    response_obs = query_response.returns[0];
    assert(response_obs.category == "unupdated_category");

    updating_obs = create_obs_instance("update_entry_test");
    updating_obs.adding.HEADER.UID = obj_return.UID;
    updating_obs.adding.category = "update_category";
    push_to_db_srv(updating_obs);

    querying = orion_actions.srv.SOMQueryObservationsRequest();
    querying.query.class_ = "update_entry_test";
    query_response:orion_actions.srv.SOMQueryObservationsResponse = get_obs_from_db_srv(querying);
    assert(len(query_response.returns) == 1);
    response_obs:orion_actions.msg.SOMObservation = query_response.returns[0];
    assert(response_obs.category == "update_category");
    print("\tTest passed");


def test_human_height_transfer():
    adding = orion_actions.srv.SOMAddObservationRequest();
    adding.adding.class_ = "person";
    adding.adding.category = "height_transfer_test";

    
    pass;


def test_observation_batch_query():
    # If the batch number queried is positive, then the specific batch will be queried.
    # If it's negative, then the test will look back that number of batches.
    print("Batch number tests.");
    obs1 = create_obs_instance("obs_1", batch_num=100);
    obs2 = create_obs_instance("obs_2", batch_num=101);
    obs3 = create_obs_instance("obs_3", batch_num=102);

    push_to_db_srv = rospy.ServiceProxy('/som/observations/input', orion_actions.srv.SOMAddObservation);
    get_obj_from_db_srv = rospy.ServiceProxy('/som/objects/basic_query', orion_actions.srv.SOMQueryObjects);

    push_to_db_srv(obs1);
    push_to_db_srv(obs2);
    push_to_db_srv(obs3);

    print("\tTesting that you can query for a specific batch number.");
    q1 = orion_actions.srv.SOMQueryObjectsRequest();
    q1.query.last_observation_batch = 100;
    q1_response:orion_actions.srv.SOMQueryObjectsResponse = get_obj_from_db_srv(q1);
    assert(len(q1_response.returns) == 1);
    assert(q1_response.returns[0].class_ == "obs_1");

    print("\tTesting that you can query for an interval of batches.");
    q2 = orion_actions.srv.SOMQueryObjectsRequest();
    q2.query.last_observation_batch = -101;
    q2_response:orion_actions.srv.SOMQueryObjectsResponse = get_obj_from_db_srv(q2);
    assert(len(q2_response.returns) == 1);
    assert(q2_response.returns[0].class_ == "obs_3");

    print("\tFinished testing batch number queries.");


def test_temporal_queries():
    # The query rounds down, so we need to wait a second before running this.
    # This will only look for objects later than the time given in the query.
    rospy.sleep(1);

    print("Temporal queries");
    query_time = rospy.Time.now();
    push_to_db_srv = rospy.ServiceProxy('/som/observations/input', orion_actions.srv.SOMAddObservation);
    get_obj_from_db_srv = rospy.ServiceProxy('/som/objects/basic_query', orion_actions.srv.SOMQueryObjects);

    rospy.sleep(0.2);

    obs1 = create_obs_instance("temporal_obs_1");
    push_to_db_srv(obs1);

    q1 = orion_actions.srv.SOMQueryObjectsRequest();
    print("\ttime before assignment = ", q1.query.last_observed_at);
    q1.query.last_observed_at = query_time;
    print("\ttime after assignment  = ", q1.query.last_observed_at);

    response:orion_actions.srv.SOMQueryObjectsResponse = get_obj_from_db_srv(q1);
    assert(len(response.returns) == 1);
    assert(response.returns[0].class_ == "temporal_obs_1");
    print("\tTemporal queries succeeded");


    pass;


def test_human_temporal_queries():
    print("Temporal queries with Humans");

    rospy.sleep(0.1);

    push_to_db_srv = rospy.ServiceProxy('/som/observations/input', orion_actions.srv.SOMAddObservation);
    get_human_observation = rospy.ServiceProxy('/som/humans/basic_query', orion_actions.srv.SOMQueryHumans);

    obs1 = create_obs_instance("person", x = 10,y=0,z=0);

    rospy.sleep(1);
    time_between_adding = rospy.Time.now();
    rospy.sleep(1);

    obs2 = create_obs_instance("person", x = -100,y=140,z=39);

    push_to_db_srv(obs1);
    obs2_return:orion_actions.srv.SOMAddObservationResponse = push_to_db_srv(obs2);

    query_1 = orion_actions.srv.SOMQueryHumansRequest();
    query_1.query.last_observed_at = time_between_adding;
    query_1_response:orion_actions.srv.SOMQueryHumansResponse = get_human_observation(query_1);
    print("\tChecking that only one response is returned.")
    assert(len(query_1_response.returns) == 1);
    return_0:orion_actions.msg.Human = query_1_response.returns[0];
    assert(return_0.obj_position.position.x == -100);
    assert(return_0.obj_position.position.y == 140);
    assert(return_0.obj_position.position.z == 39);

    print("\tTemporal queries on humans succeeded");

    
def test_input_array():
    print("Testing the inputting of arrays.")

    obj_name_prefix = "multi_input_";
    input_field = orion_actions.srv.SOMAddObservationArrRequest();

    print("\tSending in the message.")
    names = ["test1", "test2", "test3"];
    for name in names:
        observation = orion_actions.msg.SOMObservation();
        observation.class_ = obj_name_prefix + name;
        observation.observation_batch_num = 15;
        input_field.adding.append(observation);
    
    array_input = rospy.ServiceProxy('/som/observations/input_array', orion_actions.srv.SOMAddObservationArr);
    object_query = rospy.ServiceProxy('/som/objects/basic_query', orion_actions.srv.SOMQueryObjects);

    response:orion_actions.srv.SOMAddObservationArrResponse = array_input(input_field);
    print("\tAsserting as to the length of the response");
    assert(len(response.UIDs) == len(names));

    print("\tTesting that all observations have been correctly added to objects.")
    for name in names:
        query = orion_actions.srv.SOMQueryObjectsRequest();
        query.query.class_ = obj_name_prefix + name;

        query_response:orion_actions.srv.SOMQueryObjectsResponse = object_query(query);
        assert(len(query_response.returns) == 1);

    print("\tArray inputs have succeeded.");


if __name__ == '__main__':
    rospy.init_node('som_test_node');

    test_observation_input();
    test_human_observation_input();
    test_obj_relational_query();
    test_covariance_method();
    test_category_callback();
    test_updating_entry();
    test_observation_batch_query();
    test_temporal_queries();
    test_human_temporal_queries();
    test_input_array();

    uid_input_test();
    

    t = rospy.Time.now();
    print(t.secs);
    print(t.nsecs);
