#!/usr/bin/env python3
import unittest
import rospy

import geometry_msgs.msg as geom_msg

import orion_actions.srv as act_srv
import orion_actions.msg as act_msg
from useful_test_funcs import skipTest, create_obs_instance

import std_srvs.srv as std_srv
import std_msgs.msg as std_msg
import numpy as np
import time


class TestObservationInputDistanceUpdate(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.push_to_db_srv = rospy.ServiceProxy('/som/observations/input', act_srv.SOMAddObservation)
        cls.get_obj_from_db_srv = rospy.ServiceProxy('/som/objects/basic_query', act_srv.SOMQueryObjects)
        cls.get_human_from_db_srv = rospy.ServiceProxy('/som/humans/basic_query', act_srv.SOMQueryHumans)

    def test_adding_two_close_objects_create_only_one_object(self):
        """If two observation from two different batches contains the same object within 1m of each other, there should only be one object in the db."""
        observation1 = create_obs_instance("close_object", 0.1, 0, 0, batch_num=0, category="vessel")
        self.push_to_db_srv(observation1)
        observation2 = create_obs_instance("close_object", 0.25, 0, 0, batch_num=1, category="vessel")
        self.push_to_db_srv(observation2)

        querying = act_srv.SOMQueryObjectsRequest()
        querying.query.class_ = "close_object"
        query_return: act_srv.SOMQueryObjectsResponse = self.get_obj_from_db_srv(querying)

        self.assertEqual(len(query_return.returns), 1, 'The last observation should have updated the object already present.')
    
    def test_adding_two_observation_with_same_object_far_away_creates_two_objects(self):
        """If two observation from two different batches contain the same object but they are further than 1m of each other, they will be considered different objects."""
        adding = create_obs_instance("distant_object", 0.1, 0, 0, batch_num=0, category="vessel")
        self.push_to_db_srv(adding)
        adding = create_obs_instance("distant_object", 1.25, 0, 0, batch_num=1, category="vessel")
        self.push_to_db_srv(adding)

        querying = act_srv.SOMQueryObjectsRequest()
        querying.query.class_ = "distant_object"
        query_return: act_srv.SOMQueryObjectsResponse = self.get_obj_from_db_srv(querying)

        self.assertEqual(len(query_return.returns), 2, 'Two distant observations should create two different objects.')

    def test_max_distance_for_updating_person_is_different_from_objects(self):
        """If two persons are added, and they are distant at least 0.5m but closer than 1m, they should still be considered different people."""
        rospy.sleep(1)
        temporal_arg = rospy.Time.from_sec(time.time()) # Time used to get only the humans created in this test

        adding = create_obs_instance("person", 0,2,0, batch_num=0)
        _ = self.push_to_db_srv(adding)

        # Note that this is within the 1m distance for a object but further away than the 0.5m distance
        # given for persons.
        adding = create_obs_instance("person", 0, 2.7, 0, batch_num=1)
        _ = self.push_to_db_srv(adding)

        query = act_srv.SOMQueryHumansRequest()
        query.query.last_observed_at = temporal_arg
        query_response: act_srv.SOMQueryHumansResponse = self.get_human_from_db_srv(query)
        self.assertEqual(len(query_response.returns), 2, 'The db should contain two separate humans.')

        querying = act_srv.SOMQueryObjectsRequest()
        querying.query.class_ = "person"
        querying.query.last_observed_at = temporal_arg
        query_return: act_srv.SOMQueryObjectsResponse = self.get_obj_from_db_srv(querying)

        self.assertEqual(len(query_return.returns), 2, 'The person should not have updated the old object.')

    def test_adding_two_close_people_create_only_one_human(self):
        """If two observation from two different batches contains the two people closer than 0.5m of each other, there should only be one object and one human in the db."""
        rospy.sleep(1)
        temporal_arg = rospy.Time.from_sec(time.time()) # Time used to get only the humans created in this test
        
        pos1 = [190, 195, -456]
        pos2 = [190, 195, -456.4]

        obs1 = create_obs_instance("person", x=pos1[0], y=pos1[1], z=pos1[2], batch_num=0)
        self.push_to_db_srv(obs1)

        obs2 = create_obs_instance("person", x=pos2[0], y=pos2[1], z=pos2[2], batch_num=1)
        self.push_to_db_srv(obs2)

        query = act_srv.SOMQueryHumansRequest()
        query.query.last_observed_at = temporal_arg
        query_response: act_srv.SOMQueryHumansResponse = self.get_human_from_db_srv(query)

        self.assertEqual(len(query_response.returns), 1, 'Only one human should have been added to the db.')

        querying = act_srv.SOMQueryObjectsRequest()
        querying.query.class_ = "person"
        querying.query.last_observed_at = temporal_arg
        query_return: act_srv.SOMQueryObjectsResponse = self.get_obj_from_db_srv(querying)

        self.assertEqual(len(query_return.returns), 1, 'Only one person object should have been added to the db.')




class TestHumanObservationInput(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.human_input = rospy.ServiceProxy('/som/human_observations/input', act_srv.SOMAddHumanObs)
        cls.human_query = rospy.ServiceProxy('/som/humans/basic_query', act_srv.SOMQueryHumans)

        human_1 = act_srv.SOMAddHumanObsRequest()
        human_1.adding.task_role = "Operator"
        human_1.adding.object_uid = "test_uid_1"
        human_1.adding.obj_position.position.x = 1
        human_1.adding.obj_position.position.y = 2
        human_1.adding.obj_position.position.z = 3
        human_1.adding.observed_at = rospy.Time.from_sec(time.time())
        _ = cls.human_input(human_1)

        rospy.sleep(0.1)

        human_2 = act_srv.SOMAddHumanObsRequest()
        human_2.adding.task_role = "Operator"
        human_2.adding.object_uid = "test_uid_2"
        human_2.adding.obj_position.position.x = 1
        human_2.adding.obj_position.position.y = 4
        human_2.adding.obj_position.position.z = 3
        human_2.adding.observed_at = rospy.Time.from_sec(time.time())
        _ = cls.human_input(human_2)
        
        human_3: act_srv.SOMAddHumanObsRequest = act_srv.SOMAddHumanObsRequest()
        human_3.adding.task_role = "unknown"
        human_3.adding.object_uid = "test_uid_3"
        human_3.adding.obj_position.position.x = 1
        human_3.adding.obj_position.position.y = 5
        human_3.adding.obj_position.position.z = 3
        human_3.adding.observed_at = rospy.Time.from_sec(time.time())
        _ = cls.human_input(human_3)

    def test_querying_specific_task_role_returns_humans_with_that_role(self):
        human_query_in = act_srv.SOMQueryHumansRequest()
        human_query_in.query.task_role = "Operator"

        response: act_srv.SOMQueryHumansResponse = self.human_query(human_query_in)

        self.assertEqual(len(response.returns), 2, 'There should be two humans with "Operator" role.')

        for human in response.returns:
            human: act_msg.Human
            self.assertEqual(human.task_role, "Operator", 'Returned humans should have the queried role.')

    def test_empty_human_query_returns_all_humans(self):
        human_empty_query = act_srv.SOMQueryHumansRequest()
        
        response: act_srv.SOMQueryHumansResponse = self.human_query(human_empty_query)

        self.assertGreater(len(response.returns), 0, 'Empty query should return at least one human.')

class TestObjectRelationalQuery(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.push_to_db_srv = rospy.ServiceProxy('/som/observations/input', act_srv.SOMAddObservation)
        cls.relational_query_srv = rospy.ServiceProxy('/som/objects/relational_query', act_srv.SOMRelObjQuery)

        obj1 = create_obs_instance("window_rel", 1, 0, 0.2)
        obj1.adding.category = "a"
        cls.push_to_db_srv(obj1)

        obj2 = create_obs_instance("banana_rel", 1.5, 0.2, 0)
        obj2.adding.category = "b"
        cls.push_to_db_srv(obj2)

        obj1 = create_obs_instance("floor_rel", 1, 0, -0.2)
        obj1.adding.category = "a"
        cls.push_to_db_srv(obj1)

    def test_relations_are_correct(self):
        query = act_srv.SOMRelObjQueryRequest()
        query.obj1.class_ = "window_rel"
        query.obj2.class_ = "banana_rel"
        query.current_robot_pose = geom_msg.Pose()
        query_output: act_srv.SOMRelObjQueryResponse = self.relational_query_srv(query)

        # Note that here the banana is in front of the window. Also, the banana is to the right of the window 
        # (Construct a set of cartesian coordinates to show this.)
        # There should also be only one output (given that there are only these two objects in position at the
        # moment.)
        self.assertEqual(len(query_output.matches), 1, 'There should only be one match.')

        match0: act_msg.Match = query_output.matches[0]

        self.assertTrue(match0.relation.near, '"window" and "banana" should be near each other.')
        self.assertTrue(match0.relation.right, '"banana" should be to the right of "window".')
        self.assertFalse(match0.relation.left, '"banana" should not be to the left of "window".')
        self.assertTrue(match0.relation.frontof, '"banana" should be in front of "window".')
        self.assertFalse(match0.relation.behind, '"banana" should not be behind "window".')
        self.assertTrue(match0.relation.above, '"banana" should be above "window".')
        self.assertFalse(match0.relation.below, '"banana" should not be below "window".')

    def test_querying_for_relation_return_only_objects_with_that_relation(self):
        query = act_srv.SOMRelObjQueryRequest()
        query.obj1.class_ = "window_rel"
        query.relation.behind = True
        query_output: act_srv.SOMRelObjQueryResponse = self.relational_query_srv(query)
        
        for element in query_output.matches:
            element: act_msg.Match
            self.assertTrue(element.relation.behind, "Elements returned should have the queried relation.")

    def test_querying_by_category_and_relation_gives_correct_objects(self):
        query = act_srv.SOMRelObjQueryRequest()
        query.obj1.category = "a"
        query.obj2.category = "b"
        query.relation.above = True
        query.current_robot_pose = geom_msg.Pose()
        query_output: act_srv.SOMRelObjQueryResponse = self.relational_query_srv(query)

        self.assertEqual(len(query_output.matches), 1, 'There should only be one match.')
        
        match: act_msg.Match = query_output.matches[0]
        self.assertEqual(match.obj1.class_, 'window_rel', 'First object should be "window".')
        self.assertEqual(match.obj2.class_, "banana_rel", 'Second object should be "banana".')


class TestUidInput(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.push_to_db_srv = rospy.ServiceProxy('/som/observations/input', act_srv.SOMAddObservation)
        cls.get_obj_from_db_srv = rospy.ServiceProxy('/som/objects/basic_query', act_srv.SOMQueryObjects)

    def test_uid_queries_return_correct_object(self):
        adding = create_obs_instance("uid_test_input_obj", 0.1, 0, 0, batch_num=0)
        obj_return: act_srv.SOMAddObservationResponse = self.push_to_db_srv(adding)

        querying = act_srv.SOMQueryObjectsRequest()
        querying.query.HEADER.UID = obj_return.UID
        query_return: act_srv.SOMQueryObjectsResponse = self.get_obj_from_db_srv(querying)
    
        self.assertEqual(len(query_return.returns), 1, 'There should be one object with given UID.')
        self.assertEqual(query_return.returns[0].HEADER.UID, obj_return.UID, 'UID should be equal to the requested one.')

    @skipTest('Can only run successfully if current session number is 1.')
    def test_querying_by_session_number_return_list_sorted_by_latest_batch_number(self):
        obs_1 = create_obs_instance("uid_test_input_obj", 0.1, 0, 0, batch_num=0)
        self.push_to_db_srv(obs_1)

        obs_2 = create_obs_instance("uid_test_input_obj", 0.1, 0, 0, batch_num=1)
        self.push_to_db_srv(obs_2)

        querying = act_srv.SOMQueryObjectsRequest()
        querying.query.HEADER.SESSION_NUM = 1
        query_return: act_srv.SOMQueryObjectsResponse = self.get_obj_from_db_srv(querying)

        self.assertGreater(len(query_return.returns), 0, 'Querying by session number should return elements added in the session')
        return_batch_nums = [element.last_observation_batch for element in query_return.returns]
        
        for i in range(len(return_batch_nums) - 1):
            self.assertLessEqual(return_batch_nums[i], return_batch_nums[i+1], 'Objects returned should be sorted by last_onservation_batch.')

class TestCategoryCallback(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.push_to_db_srv = rospy.ServiceProxy('/som/observations/input', act_srv.SOMAddObservation)
        cls.get_obj_from_db_srv = rospy.ServiceProxy('/som/objects/basic_query', act_srv.SOMQueryObjects)

    def test_category_is_inferred_correctly(self):
        adding = create_obs_instance("food_tray", 0,0,1)
        self.push_to_db_srv(adding)

        querying = act_srv.SOMQueryObjectsRequest()
        querying.query.class_ = "food_tray"
        query_return: act_srv.SOMQueryObjectsResponse = self.get_obj_from_db_srv(querying)
        
        self.assertGreater(len(query_return.returns), 0, 'The object should have been added to the database.')
        
        obj: act_msg.SOMObject = query_return.returns[0]
        self.assertEqual(obj.category, "containers", 'The catagory assigned to the object is not the one inferred from the class.')

class TestCovarianceMethod(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.push_to_db_srv = rospy.ServiceProxy('/som/observations/input', act_srv.SOMAddObservation)
        cls.get_obj_from_db_srv = rospy.ServiceProxy('/som/objects/basic_query', act_srv.SOMQueryObjects)

    def test_covariance_raises_no_exception(self):
        obj_name = "covariance_test"
        
        adding_1 = create_obs_instance(obj_name, 0.1,0,0, 2)
        adding_1.adding.covariance_mat = [1,0,0,0,10,0,0,0,1]
        self.push_to_db_srv(adding_1)
        
        adding_2 = create_obs_instance(obj_name, 0,0.1,0, 3)
        adding_2.adding.covariance_mat = [10,0,0,0,1,0,0,0,1]
        self.push_to_db_srv(adding_2)

        query = act_srv.SOMQueryObjectsRequest()
        query.query.class_ = obj_name
        query_response: act_srv.SOMQueryObjectsResponse = self.get_obj_from_db_srv(query)
        
class TestUpdatingEntry(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.push_to_db_srv = rospy.ServiceProxy('/som/observations/input', act_srv.SOMAddObservation)
        cls.get_obs_from_db_srv = rospy.ServiceProxy('/som/observations/basic_query', act_srv.SOMQueryObservations)

    def test_updating_entry_works_correctly(self):
        category_before_update = 'unupdated_category'
        category_after_update = 'updated_category'
        entry_class = 'update_entry_test'

        adding = create_obs_instance(entry_class, 0.1, 0, 0, batch_num=0, category=category_before_update)
        obj_return: act_srv.SOMAddObservationResponse = self.push_to_db_srv(adding)
        
        updating_obs = create_obs_instance(entry_class)
        updating_obs.adding.HEADER.UID = obj_return.UID
        updating_obs.adding.category = category_after_update
        self.push_to_db_srv(updating_obs)

        querying = act_srv.SOMQueryObservationsRequest()
        querying.query.class_ = entry_class
        query_response: act_srv.SOMQueryObservationsResponse = self.get_obs_from_db_srv(querying)

        self.assertEqual(len(query_response.returns), 1, 'There should only be one entry in the database after updating.')
        response_obs: act_msg.SOMObservation = query_response.returns[0]
        self.assertEqual(response_obs.category, category_after_update, 'The category after updating is not the corect one.')

class TestRetrievingObservationByBatchNumber(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.obs_batch_100 = create_obs_instance("obs_1", batch_num=100)
        cls.obs_batch_101 = create_obs_instance("obs_2", batch_num=101)
        cls.obs_batch_102 = create_obs_instance("obs_3", batch_num=102)

        cls.push_to_db_srv = rospy.ServiceProxy('/som/observations/input', act_srv.SOMAddObservation)
        cls.get_obj_from_db_srv = rospy.ServiceProxy('/som/objects/basic_query', act_srv.SOMQueryObjects)

        cls.push_to_db_srv(cls.obs_batch_100)
        cls.push_to_db_srv(cls.obs_batch_101)
        cls.push_to_db_srv(cls.obs_batch_102)

    def test_query_positive_batch_returns_only_that_batch(self):
        """If the batch number queried is positive, then the specific batch will be queried."""
        q1 = act_srv.SOMQueryObjectsRequest()
        q1.query.last_observation_batch = 100
        q1_response: act_srv.SOMQueryObjectsResponse = self.get_obj_from_db_srv(q1)

        self.assertEqual(len(q1_response.returns), 1, 'There should only be one object of batch 100.')
        self.assertEqual(q1_response.returns[0].class_, self.obs_batch_100.adding.class_, 'The object returned is not the one from the correct batch.')

    def test_query_interval_of_batches_returns_correct_value(self):
        """If the batch number queried is negative, then the test will look back that number of batches."""
        q2 = act_srv.SOMQueryObjectsRequest()
        q2.query.last_observation_batch = -101
        q2_response: act_srv.SOMQueryObjectsResponse = self.get_obj_from_db_srv(q2)
        self.assertEqual(len(q2_response.returns), 1, 'There should be one observation in this range.')
        self.assertEqual(q2_response.returns[0].class_, self.obs_batch_102.adding.class_, 'The object returned is not the one from the correct batch.')

class TestTemporalQueries(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.push_to_db_srv = rospy.ServiceProxy('/som/observations/input', act_srv.SOMAddObservation)
        cls.get_obj_from_db_srv = rospy.ServiceProxy('/som/objects/basic_query', act_srv.SOMQueryObjects)
        cls.get_human_observation = rospy.ServiceProxy('/som/humans/basic_query', act_srv.SOMQueryHumans)

    def test_querying_objects_with_time_returns_only_objects_seen_later(self):
        past_obs = create_obs_instance("temporal_obs_1")
        self.push_to_db_srv(past_obs)

        # The query rounds down, so we need to wait a second before running this.
        # This will only look for objects later than the time given in the query.
        rospy.sleep(1)
        time_new_obs = rospy.Time.from_sec(time.time())
        new_obs = create_obs_instance("temporal_obs_2")
        self.push_to_db_srv(new_obs)

        q1 = act_srv.SOMQueryObjectsRequest()
        q1.query.last_observed_at = time_new_obs
        
        response: act_srv.SOMQueryObjectsResponse = self.get_obj_from_db_srv(q1)

        self.assertEqual(len(response.returns), 1, 'Only one object should have been created since given time.')
        self.assertEqual(response.returns[0].class_, new_obs.adding.class_, 'The object returned is not the one created after the given time.')

    def test_querying_humans_with_time_returns_only_humans_seen_later(self):
        obs1 = create_obs_instance("person", x=10, y=0, z=0)

        rospy.sleep(1)
        time_between_adding = rospy.Time.from_sec(time.time())

        obs2_position = [-100, 140, 39]
        obs2 = create_obs_instance("person", x=obs2_position[0], y=obs2_position[1], z=obs2_position[2])

        self.push_to_db_srv(obs1)
        self.push_to_db_srv(obs2)

        query_1 = act_srv.SOMQueryHumansRequest()
        query_1.query.last_observed_at = time_between_adding
        response: act_srv.SOMQueryHumansResponse = self.get_human_observation(query_1)

        self.assertEqual(len(response.returns), 1, 'Only one human was created since given time.')

        return_0: act_msg.Human = response.returns[0]
        response_position = [return_0.obj_position.position.x, return_0.obj_position.position.y, return_0.obj_position.position.z]

        self.assertListEqual(response_position, obs2_position, 'Human returned is not the one created after the given time.')

@skipTest("Testing the visual pipeline into SOM: NEEDS TF TREE TO BE RUNNING.")
class TestVisualPipeline(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.detections_pub = rospy.Publisher('/vision/bbox_detections', act_msg.DetectionArray, queue_size=10)
        cls.get_obj_from_db_srv = rospy.ServiceProxy('/som/objects/basic_query', act_srv.SOMQueryObjects)

    def test_observation_published_in_bbox_detection_is_added_to_db(self):
        header = std_msg.Header()
        header.stamp = rospy.Time.from_sec(time.time())

        score_lbl = act_msg.Label("pipeline_test", np.float64(1))
        center_x = 0
        center_y = 0
        width = 1
        height = 1
        size = geom_msg.Point(1,1,1)
        obj = [1,2,3]
        stamp = rospy.Time.from_sec(time.time())
        colour = "purple"

        detection = act_msg.Detection(score_lbl, center_x, center_y, width, height,
                        size, colour, obj[0], obj[1], obj[2], stamp)
        detectionArr = act_msg.DetectionArray(header, [detection])
        self.detections_pub.publish(detectionArr);
        for _ in range(3):
            rospy.sleep(0.1)
            detection.timestamp = rospy.Time.from_sec(time.time())
            header.stamp = rospy.Time.from_sec(time.time())
            self.detections_pub.publish(detectionArr)

        
        q1 = act_srv.SOMQueryObjectsRequest()
        q1.query.class_ = "pipeline_test"
        result: act_srv.SOMQueryObjectsResponse = self.get_obj_from_db_srv(q1)
        self.assertEqual(len(result.returns), 1, 'Only one object should have been added with this class')

@skipTest('Not enabled in current configuration.')
class TestArrayInput(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.array_input = rospy.ServiceProxy('/som/observations/input_array', act_srv.SOMAddObservationArr)
        cls.object_query = rospy.ServiceProxy('/som/objects/basic_query', act_srv.SOMQueryObjects)

    def test_all_observations_in_array_are_added(self):
        obj_name_prefix = "multi_input_"
        input_field = act_srv.SOMAddObservationArrRequest()

        names = ["test1", "test2", "test3"]
        for name in names:
            observation = act_msg.SOMObservation()
            observation.class_ = obj_name_prefix + name
            observation.observation_batch_num = 15
            observation.observed_at = rospy.Time.from_sec(time.time())
            input_field.adding.append(observation)

        response: act_srv.SOMAddObservationArrResponse = self.array_input(input_field)
        
        self.assertEqual(len(response.UIDs), len(names), 'Respose does not contains UIDs for all objects inserted.')

        for name in names:
            query = act_srv.SOMQueryObjectsRequest()
            query.query.class_ = obj_name_prefix + name

            query_response: act_srv.SOMQueryObjectsResponse = self.object_query(query)
            self.assertEqual(len(query_response.returns), 1, f'The observation {name} was not stored in object database.')

class GeneralTestsuite(unittest.TestSuite):

    def __init__(self):
        super(GeneralTestsuite, self).__init__()
        self.addTest(unittest.makeSuite(TestObservationInputDistanceUpdate))
        self.addTest(unittest.makeSuite(TestHumanObservationInput))
        self.addTest(unittest.makeSuite(TestObjectRelationalQuery))
        self.addTest(unittest.makeSuite(TestCovarianceMethod))
        self.addTest(unittest.makeSuite(TestCategoryCallback))
        self.addTest(unittest.makeSuite(TestUpdatingEntry))
        self.addTest(unittest.makeSuite(TestRetrievingObservationByBatchNumber))
        self.addTest(unittest.makeSuite(TestTemporalQueries))
        self.addTest(unittest.makeSuite(TestVisualPipeline))
        self.addTest(unittest.makeSuite(TestArrayInput))
        self.addTest(unittest.makeSuite(TestUidInput))
        

if __name__ == '__main__':
    import rostest
    rostest.rosrun('semantic_mapping', 'test_observations_input_output', 'tests.GeneralTestsuite')