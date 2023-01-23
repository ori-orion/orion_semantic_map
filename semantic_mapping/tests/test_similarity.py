#!/usr/bin/env python3

import rospy

from orion_actions.srv import SOMSimilarityQuery, SOMSimilarityQueryRequest, SOMSimilarityQueryResponse

def test_similarity(similarity_service: rospy.ServiceProxy):
    """Test similarity for two different terms is between 0 and 1"""
    print('Testing similarity between two different terms')
    
    query = SOMSimilarityQueryRequest('Sour_Candy_bag', 'Gummy_Candy_bag')
    response: SOMSimilarityQueryResponse = similarity_service(query)

    assert 0 < response.similarity < 1, f'Similarity should be between 0 and 1, received {response.similarity}'

    print('\tDone')

def test_similarity_same_object(similarity_service: rospy.ServiceProxy):
    """Test that similarity of two terms that are equal is 1"""
    print('Testing similarity when terms are equal')
    
    query = SOMSimilarityQueryRequest('Sour_Candy_bag', 'Sour_Candy_bag')
    response: SOMSimilarityQueryResponse = similarity_service(query)

    assert response.similarity == 1, f'Similarity should be 1 when two terms are equal, received {response.similarity}'
    
    print('\tDone')

def test_similarity_object_not_present(similarity_service: rospy.ServiceProxy):
    """Test that similarity is 0 if an object is not present"""
    print('Testing similarity when a term is not present')

    query1 = SOMSimilarityQueryRequest('object_not_present', 'Sour_Candy_bag')
    query2 = SOMSimilarityQueryRequest('Sour_Candy_bag', 'object_not_present')

    response1: SOMSimilarityQueryResponse = similarity_service(query1)
    response2: SOMSimilarityQueryResponse = similarity_service(query2)

    assert response1.similarity == 0, f'Similarity should be 0 when first term not present in the ontology, received {response1.similarity}'
    assert response2.similarity == 0, f'Similarity should be 0 when second term not present in the ontology, received {response2.similarity}'

    print('\tDone')


if __name__ == '__main__':
    rospy.init_node('som_test_similarity_node')

    similarity_service = rospy.ServiceProxy('/som/similarity/get_similarity', SOMSimilarityQuery)

    test_similarity(similarity_service)
    test_similarity_same_object(similarity_service)
    test_similarity_object_not_present(similarity_service)

