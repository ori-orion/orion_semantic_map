import rospy

from orion_actions.srv import SOMSimilarityQuery, SOMSimilarityQueryResponse, SOMSimilarityQueryRequest

from MemoryManager import SERVICE_ROOT
from OntologyTree import OntologyTree


class OntologySimilarityManager:
    def __init__(self, filename: str, service_name: str):
        """
        Creates a service for computing similarity between two terms. 

        - `filename` is the file from which the ontology tree should be loaded
        - `service_name` is the name used by the service
        """
        self.ontology_tree = OntologyTree.from_file(filename)
        self.service_name = service_name

        self.setup_service()

    def compute_similarity_callback(self, msg: SOMSimilarityQueryRequest) -> SOMSimilarityQueryResponse:
        """Callback executed when a message is received. It computes the similarity of two terms"""
        similarity = self.ontology_tree.similarity(msg.term1, msg.term2)
        
        return SOMSimilarityQueryResponse(similarity)

    def setup_service(self):
        """Create the service"""
        rospy.Service(
            SERVICE_ROOT + self.service_name + '/get_similarity',
            SOMSimilarityQuery,
            self.compute_similarity_callback)


if __name__ == '__main__':
    manager = OntologySimilarityManager('./taxonomyLabels.txt', 'similarity_srv')

    message = SOMSimilarityQueryRequest('Sour_Candy_bag', 'Gummy_Candy_bag')
    print(manager.compute_similarity_callback(message))
    