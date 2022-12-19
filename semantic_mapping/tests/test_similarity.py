#!/usr/bin/env python3

import rospy
from orion_actions.srv import SOMSimilarityQuery, SOMSimilarityQueryRequest, SOMSimilarityQueryResponse

import unittest

class TestOntologySimilarity(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.similarity_service = rospy.ServiceProxy('/som/similarity/get_similarity', SOMSimilarityQuery)

    def test_similarity_is_between_zero_and_one(self):
        """Test that similarity for two different terms is between 0 and 1."""
        
        query = SOMSimilarityQueryRequest('Sour_Candy_bag', 'Gummy_Candy_bag')
        response: SOMSimilarityQueryResponse = self.similarity_service(query)

        self.assertGreater(response.similarity, 0, 'Similarity should be greater than 0 for two different terms.')
        self.assertLess(response.similarity, 1, 'Similarity should be less than 1 for two different terms.')

    def test_similarity_same_terms_is_one(self):
        """Test that similarity of two terms that are equal is 1."""
        
        query = SOMSimilarityQueryRequest('Sour_Candy_bag', 'Sour_Candy_bag')
        response: SOMSimilarityQueryResponse = self.similarity_service(query)

        self.assertEqual(response.similarity, 1, 'Similarity should be 1 when two terms are equal.')
        
    def test_similarity_with_unknown_term_is_zero(self):
        """Test that similarity is 0 if one of the terms is not present in the ontology."""

        query1 = SOMSimilarityQueryRequest('object_not_present', 'Sour_Candy_bag')
        query2 = SOMSimilarityQueryRequest('Sour_Candy_bag', 'object_not_present')

        response1: SOMSimilarityQueryResponse = self.similarity_service(query1)
        response2: SOMSimilarityQueryResponse = self.similarity_service(query2)

        self.assertEqual(response1.similarity, 0, 'Similarity should be 0 when first term not present in the ontology')
        self.assertEqual(response2.similarity, 0, 'Similarity should be 0 when second term not present in the ontology')

if __name__ == '__main__':
    import rostest
    rostest.rosrun('semantic_mapping', 'test_ontology_similarity', 'test_similarity.TestOntologySimilarity')
