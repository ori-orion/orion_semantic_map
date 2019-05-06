import unittest
import rospy
from semantic_mapping.som_manager import SOMDataManager


class TestDatabase(unittest.TestCase):

    def setUp(self):
        self.db_manager = SOMDataManager('../tests/config/test_ontology.owl', '../tests/config/test_rooms.pkl', 'True')
        rospy.wait_for_service('som/observe')
        rospy.wait_for_service('som/lookup')
        rospy.wait_for_service('som/query')

        observe_objs_srv = rospy.ServiceProxy('som/observe', SOMObserve)
        lookup_object_srv = rospy.ServiceProxy('som/lookup', SOMLookup)
        delete_object_srv = rospy.ServiceProxy('som/delete', SOMDelete)
        query_object_srv = rospy.ServiceProxy('som/query', SOMQuery)
        print("hello")

    def tearDown(self):
        pass

    def test_something(self):
        self.assertEqual(5,5)
        return

if __name__ == '__main__':
    unittest.main()
