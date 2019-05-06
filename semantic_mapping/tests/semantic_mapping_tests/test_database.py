import unittest
import rospy
from semantic_mapping.som_manager import SOMDataManager


class TestDatabase(unittest.TestCase):

    def setUp(self):
        rospy.wait_for_service('som/observe')
        rospy.wait_for_service('som/lookup')
        rospy.wait_for_service('som/delete')
        rospy.wait_for_service('som/query')
        rospy.wait_for_service('som/clear_database')

        self.observe_objs_srv = rospy.ServiceProxy('som/observe', SOMObserve)
        self.lookup_object_srv = rospy.ServiceProxy('som/lookup', SOMLookup)
        self.delete_object_srv = rospy.ServiceProxy('som/delete', SOMDelete)
        self.query_object_srv = rospy.ServiceProxy('som/query', SOMQuery)
        self.clear_database_srv = rospy.ServiceProxy('som/clear_database', SOMClearDatabase)
        self.clear_database_srv()

    def tearDown(self):
        pass

    def test_something(self):
        self.assertEqual(5,5)
        return

if __name__ == '__main__':
    unittest.main()
