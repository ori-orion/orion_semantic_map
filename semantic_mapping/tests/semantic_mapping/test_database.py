import unittest
import rospy
from semantic_mapping.som_manager import SOMDataManager
from semantic_mapping.srv import *
from semantic_mapping.msg import *

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

    def tearDown(self):
        pass

    def test_add_to_database(self):
        obs = SOMObservation()
        obs.type = "pizza"
        obs.colour = "red"
        self.observe_objs_srv(obs)
        objects = self.get_all_objects_srv().objects

        self.assertEqual(len(objects), 1)
        self.assertEqual(objects[0].colour, "red")

    def test_clear_database(self):
        obs = SOMObservation()
        obs.type = "pizza"
        obs.colour = "red"
        self.observe_objs_srv(obs)
        obs2 = SOMObservation()
        obs2.type = "boy"
        self.observe_objs_srv(obs2)

        objects = self.get_all_objects_srv().objects
        self.assertEqual(len(objects), 2)
        self.clear_database_srv()
        objects = self.get_all_objects_srv().objects
        self.assertEqual(len(objects), 0)

    def test_lookup_by_id(self):
        obs = SOMObservation()
        obs.type = "bacon"
        obs.colour = "red"
        response1 = self.observe_objs_srv(obs)
        obj1_id = response1.obj_id

        obs = SOMObservation()
        obs.type = "pizza"
        obs.colour = "blue"
        response2 = self.observe_objs_srv(obs)
        obj2_id = response2.obj_id

        lookup_resp = self.lookup_object_srv(obj1_id)
        obj1 = lookup_resp.object
        self.assertEqual(obj1.obj_id, obj1_id)
        self.assertEqual(obj1.type, 'bacon')
        self.assertEqual(obj1.colour, 'red')

        lookup_resp = self.lookup_object_srv(obj2_id)
        obj2 = lookup_resp.object
        self.assertEqual(obj2.obj_id, obj2_id)
        self.assertEqual(obj2.type, 'pizza')
        self.assertEqual(obj2.colour, 'blue')

    def test_update_by_id(self):
        obs = SOMObservation()
        obs.type = "bacon"
        obs.colour = "red"
        response1 = self.observe_objs_srv(obs)
        obj1_id = response1.obj_id

        obs = SOMObservation()
        obs.type = "pizza"
        obs.colour = "blue"
        response2 = self.observe_objs_srv(obs)
        obj2_id = response2.obj_id

        obs = SOMObservation()
        obs.colour = "green"
        obs.age = 40
        obs.obj_id = obj1_id
        self.observe_objs_srv(obs)

        objects = self.get_all_objects_srv().objects
        self.assertEqual(len(objects), 2)

        lookup_resp = self.lookup_object_srv(obj1_id)
        obj1 = lookup_resp.object
        self.assertEqual(obj1.type, 'bacon')
        self.assertEqual(obj1.colour, 'green')
        self.assertEqual(obj1.age, 40)

    def test_delete_by_id(self):
        obs = SOMObservation()
        obs.type = "bacon"
        obs.colour = "red"
        response1 = self.observe_objs_srv(obs)
        obj1_id = response1.obj_id

        obs = SOMObservation()
        obs.type = "pizza"
        obs.colour = "blue"
        response2 = self.observe_objs_srv(obs)
        obj2_id = response2.obj_id
        objects = self.get_all_objects_srv().objects
        self.assertEqual(len(objects), 2)

        self.delete_object_srv(obj1_id)
        objects = self.get_all_objects_srv().objects
        self.assertEqual(len(objects), 1)
        self.assertEqual(objects[0].type, 'pizza')

        self.delete_object_srv(obj2_id)
        objects = self.get_all_objects_srv().objects
        self.assertEqual(len(objects), 0)

if __name__ == '__main__':
    unittest.main()
