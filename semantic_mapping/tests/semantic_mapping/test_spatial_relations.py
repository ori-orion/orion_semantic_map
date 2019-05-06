import unittest
import rospy
from geometry_msgs.msg import Point, Pose
from semantic_mapping.som_manager import SOMDataManager
from semantic_mapping.queries import spatial_relation
from semantic_mapping.srv import *
from semantic_mapping.msg import *

class TestSpatialRelations(unittest.TestCase):

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
        obs1.pose_observation.position = Point(0.0, 1.0, 0.0)
        obs1.type = 'Pizza'
        response1 = self.observe_objs_srv(obs1)
        lookup_resp = self.lookup_object_srv(response1.obj_id)
        self.pizza = lookup_resp.object

        obs2 = SOMObservation()
        obs2.pose_observation.position = Point(1.0, 1.0, 0.0)
        obs2.type = 'Bacon'
        response2 = self.observe_objs_srv(obs2)
        lookup_resp = self.lookup_object_srv(response2.obj_id)
        self.bacon = lookup_resp.object

        obs3 = SOMObservation()
        obs3.pose_observation.position = Point(2.0, 2.0, 0.0)
        obs3.type = 'Boy'
        response3 = self.observe_objs_srv(obs3)
        lookup_resp = self.lookup_object_srv(response3.obj_id)
        self.boy = lookup_resp.object

        obs4 = SOMObservation()
        obs4.pose_observation.position = Point(-1.0, 1.0, 0.0)
        obs4.type = 'Girl'
        response4 = self.observe_objs_srv(obs4)
        lookup_resp = self.lookup_object_srv(response4.obj_id)
        self.girl = lookup_resp.object

        obs5 = SOMObservation()
        obs5.pose_observation.position = Point(0.0, 1.0, 1.0)
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

        obs7 = SOMObservation()
        obs7.pose_observation.position = Point(0.0, 2.0, 0.0)
        obs7.type = 'Man'
        response7 = self.observe_objs_srv(obs7)
        lookup_resp = self.lookup_object_srv(response7.obj_id)
        self.man = lookup_resp.object

    def tearDown(self):
        self.clear_database_srv()

    def test_left_right(self):
        relation = spatial_relation(self.robot_pose, self.girl, self.pizza)
        self.assertEqual(relation.left, True)
        self.assertEqual(relation.right, False)
        self.assertEqual(relation.frontof, False)
        self.assertEqual(relation.near, True)

    def test_near_far(self):
        relation = spatial_relation(self.robot_pose, self.milk, self.pizza)
        self.assertEqual(relation.near, False)
        relation = spatial_relation(self.robot_pose, self.juice, self.pizza)
        self.assertEqual(relation.near, True)

    def test_behind(self):
        relation = spatial_relation(self.robot_pose, self.man, self.pizza)
        self.assertEqual(relation.behind, True)
        relation = spatial_relation(self.robot_pose, self.pizza, self.girl)
        self.assertEqual(relation.behind, False)

    def test_frontof(self):
        relation = spatial_relation(self.robot_pose, self.man, self.pizza)
        self.assertEqual(relation.frontof, False)
        relation = spatial_relation(self.robot_pose, self.bacon, self.boy)
        self.assertEqual(relation.frontof, True)

    def test_above_below(self):
        relation = spatial_relation(self.robot_pose, self.juice, self.pizza)
        self.assertEqual(relation.above, True)
        self.assertEqual(relation.below, False)
        relation = spatial_relation(self.robot_pose, self.pizza, self.juice)
        self.assertEqual(relation.above, False)
        self.assertEqual(relation.below, True)

if __name__ == '__main__':
    unittest.main()
