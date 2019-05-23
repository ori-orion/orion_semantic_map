import unittest
import rospy
from geometry_msgs.msg import Point, Pose
from semantic_mapping.som_manager import SOMDataManager
from semantic_mapping.queries import spatial_relation
from orion_actions.srv import *
from orion_actions.msg import *

class TestDatabase(unittest.TestCase):

    def setUp(self):
        rospy.wait_for_service('som/observe')
        rospy.wait_for_service('som/lookup')
        rospy.wait_for_service('som/delete')
        rospy.wait_for_service('som/query')
        rospy.wait_for_service('som/clear_database')
        rospy.wait_for_service('som/get_all_objects')

        self.observe_objs_srv = rospy.ServiceProxy('som/observe', SOMObserve)
        self.lookup_object_srv = rospy.ServiceProxy('som/lookup', SOMLookup)
        self.delete_object_srv = rospy.ServiceProxy('som/delete', SOMDelete)
        self.query_object_srv = rospy.ServiceProxy('som/query', SOMQuery)
        self.clear_database_srv = rospy.ServiceProxy('som/clear_database', SOMClearDatabase)
        self.get_all_objects_srv = rospy.ServiceProxy('som/get_all_objects', SOMGetAllObjects)
        self.clear_database_srv()

        self.robot_pose = Pose()

        obs1 = SOMObservation()
        obs1.pose_observation.position = Point(0.4, 1.0, 0.0)
        obs1.type = 'Pizza'
        response1 = self.observe_objs_srv(obs1)
        lookup_resp = self.lookup_object_srv(response1.obj_id)
        self.pizza_office = lookup_resp.object

        obs2 = SOMObservation()
        obs2.pose_observation.position = Point(1.0, 1.0, 0.0)
        obs2.type = 'Bacon'
        response2 = self.observe_objs_srv(obs2)
        lookup_resp = self.lookup_object_srv(response2.obj_id)
        self.bacon = lookup_resp.object

        obs4 = SOMObservation()
        obs4.pose_observation.position = Point(-1.0, 1.0, 0.0)
        obs4.type = 'Girl'
        response4 = self.observe_objs_srv(obs4)
        lookup_resp = self.lookup_object_srv(response4.obj_id)
        self.girl = lookup_resp.object

        obs5 = SOMObservation()
        obs5.pose_observation.position = Point(0.4, 1.0, 1.0)
        obs5.type = 'Juice'
        response5 = self.observe_objs_srv(obs5)
        lookup_resp = self.lookup_object_srv(response5.obj_id)
        self.juice = lookup_resp.object

        obs6 = SOMObservation()
        obs6.pose_observation.position = Point(-6.0, -6.0, 1.0)
        obs6.type = 'Milk'
        response6 = self.observe_objs_srv(obs6)
        lookup_resp = self.lookup_object_srv(response6.obj_id)
        self.milk = lookup_resp.object

        obs8 = SOMObservation()
        obs8.pose_observation.position = Point(0.35, -3.0, 0.0)
        obs8.type = 'Pizza'
        response8 = self.observe_objs_srv(obs8)
        lookup_resp = self.lookup_object_srv(response8.obj_id)
        self.pizza_bedroom = lookup_resp.object

        obs9 = SOMObservation()
        obs9.pose_observation.position = Point(0.35, -3.0, 1.0)
        obs9.type = 'Milk'
        response9 = self.observe_objs_srv(obs9)
        lookup_resp = self.lookup_object_srv(response9.obj_id)
        self.milk_bedroom = lookup_resp.object

    def tearDown(self):
        pass

    def test_room_allocation(self):
        self.assertEqual(self.milk_bedroom.room_name, 'Bedroom')
        self.assertEqual(self.pizza_bedroom.room_name, 'Bedroom')
        self.assertEqual(self.bacon.room_name, 'Office')
        self.assertEqual(self.girl.room_name, 'Bathroom')
        self.assertEqual(self.milk.room_name, 'NotInRoom')

    def test_unobserved_object_query(self):
        query = SOMObservation()
        query.type = 'boy'
        resp = self.query_object_srv(query, Relation(), SOMObservation(), Pose())
        print(resp.matches[0].obj1.pose_estimate)

    def test_single_object_query(self):
        query = SOMObservation()
        query.type = 'pizza'
        resp = self.query_object_srv(query, Relation(), SOMObservation(), Pose())
        self.assertEqual(len(resp.matches), 2)

        query = SOMObservation()
        query.type = 'pizza'
        query.room_name = 'Bedroom'
        resp = self.query_object_srv(query, Relation(), SOMObservation(), Pose())
        self.assertEqual(len(resp.matches), 1)
        self.assertEqual(resp.matches[0].obj1.type.lower(), 'pizza')
        self.assertEqual(resp.matches[0].obj2.type.lower(), '')

    def test_object_and_relation(self):
        query = SOMObservation()
        query.type = 'pizza'
        query.room_name = 'Bedroom'
        resp = self.query_object_srv(SOMObservation(),  Relation(above = True), query, Pose())
        self.assertEqual(len(resp.matches), 1)
        self.assertEqual(resp.matches[0].obj1.type.lower(), 'milk')

        query = SOMObservation()
        query.type = 'juice'
        resp = self.query_object_srv(query,  Relation(above = True), SOMObservation(), Pose())
        self.assertTrue(len(resp.matches) >= 2 and len(resp.matches) <= 3)

    def test_object_relation_object(self):
        query = SOMObservation()
        query.type = 'pizza'
        query2 = SOMObservation()
        query2.type = 'milk'
        resp = self.query_object_srv(query, Relation(below = True), query2, Pose())
        self.assertEqual(len(resp.matches), 1)
        self.assertEqual(resp.matches[0].obj1.room_name.lower(), 'bedroom')

    def test_relation_only_query(self):
        resp = self.query_object_srv(SOMObservation(), Relation(above = True), SOMObservation(), Pose())
        self.assertTrue(len(resp.matches) >= 3 and len(resp.matches) <= 4)
        self.assertTrue(resp.matches[0].obj1.type == 'juice' or resp.matches[0].obj1.type == 'milk')
        self.assertTrue(resp.matches[1].obj1.type == 'juice' or resp.matches[1].obj1.type == 'milk')
        self.assertTrue(resp.matches[2].obj1.type == 'juice' or resp.matches[2].obj1.type == 'milk')

        self.assertTrue(resp.matches[0].obj2.type == 'pizza' or resp.matches[0].obj2.type == 'bacon' or resp.matches[0].obj2.type == 'girl')
        self.assertTrue(resp.matches[1].obj2.type == 'pizza' or resp.matches[1].obj2.type == 'bacon' or resp.matches[1].obj2.type == 'girl')
        self.assertTrue(resp.matches[2].obj2.type == 'pizza' or resp.matches[2].obj2.type == 'bacon' or resp.matches[2].obj2.type == 'girl')

if __name__ == '__main__':
    unittest.main()
