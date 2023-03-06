#!/usr/bin/env python3
"""
Note that double detection suppression needs to be enabled for this to work.
"""

import unittest
from orion_actions.srv import SOMAddObservation, SOMQueryObservations, SOMQueryObjects, \
    SOMQueryObservationsRequest, SOMQueryObservationsResponse, SOMQueryObjectsRequest, SOMQueryObjectsResponse

import rospy;

from useful_test_funcs import create_obs_instance;


class TestDoubleDetectionSuppression(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.observations_input = rospy.ServiceProxy('/som/observations/input', SOMAddObservation)
        cls.observations_query = rospy.ServiceProxy('/som/observations/basic_query', SOMQueryObservations)
        cls.objects_query = rospy.ServiceProxy('/som/objects/basic_query', SOMQueryObjects)

    def test_double_detection_suppression(self):
        """Test that double observation is ignore both in observations db and objects db."""
        suppressing_test_class = "suppression_test_type"

        obs_add_1 = create_obs_instance(suppressing_test_class)
        self.observations_input(obs_add_1)
        obs_add_2 = create_obs_instance(suppressing_test_class, x = 0.001)
        self.observations_input(obs_add_2)

        obs_query = SOMQueryObservationsRequest()
        obs_query.query.class_ = suppressing_test_class
        obs_query_response: SOMQueryObservationsResponse = self.observations_query(obs_query)
        self.assertEqual(len(obs_query_response.returns), 1, 'The second obsrevation should not have been saved.')

        obj_query = SOMQueryObjectsRequest()
        obj_query.query.class_ = suppressing_test_class
        obj_query_response:SOMQueryObjectsResponse = self.objects_query(obj_query)
        self.assertEqual(len(obj_query_response.returns), 1, 'The object should only be added once.')

if __name__ == '__main__':
    import rostest
    rostest.rosrun('semantic_mapping', 'test_ontology_similarity', 'test_similarity.TestOntologySimilarity')