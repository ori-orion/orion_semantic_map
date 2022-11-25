import os
from typing import List
from Ontology import ontology_member


class OntologyTree:
    def __init__(self, base_label='Objs'):
        """
        Create a Ontology Tree, where the root has label `base_label`.
        """
        self.root = ontology_member(base_label)
    
    @classmethod
    def from_file(cls, filename='./taxonomyLabels.txt', base_label='Obj') -> 'OntologyTree':
        """
        Load a `OntologyTree` from a file. The file should have a term for each line, with all its 
        parent categories (except for the tree base label) separated by `/`. For example:
        
        food/fruit/apple
        food/snacks/Lollipops
        ...
        """
        tree = cls(base_label)

        with open(filename) as input_file:
            for line in input_file.readlines():
                cleaned_line = line.replace('\n', '') # Remove line endings
                categories = cleaned_line.split('/')
                tree.add_term(categories)

        return tree

    def add_term(self, term_categories: List[str]):
        """
        Add the term to the tree. `term_categories` should contain the hierarchy of the term, from the 
        most general category (excluding the base label of the tree) and ending with the term to be added.
        If a category is not already present, it will be added to the tree.
        """
        self.root.add_term(term_categories)
    
    def search_term(self, term: str) -> List[str]:
        """
        Search for a term in the tree. Returns a list representing the hierarchy of categories, starting
        with the most general (the `base_label` of the tree) and ending with the term.
        If the term is not resent in the tree, returns an empty list.
        """
        result = self.root.search_for_term(term)
        
        if not result:
            result = []

        # Reorder the categories from the most genral to the most specific
        result.reverse()

        return result

    def similarity(self, term1: str, term2: str) -> float:
        """
        Computes the similarity between two terms. It uses Wu-Palmer formula, and returns
        a value between 0 and 1. 0 is returned only if one of the terms is not present in the tree.
        1 is returned when the two terms are the same.
        """
        categories1 = self.search_term(term1)
        categories2 = self.search_term(term2)

        # If one term is not present, similarity is zero
        if not categories1 or not categories2:
            return 0

        least_common_subsumer_depth = 0
        for category1, category2 in zip(categories1, categories2):
            if category1 != category2:
                break
            least_common_subsumer_depth += 1

        return 2*least_common_subsumer_depth / (len(categories1) + len(categories2))


if __name__ == '__main__':
    tree = OntologyTree.from_file(os.path.dirname(__file__) + '/taxonomyLabels.txt')
    
    print(tree.search_term('Sour_Candy_bag'))
    print(tree.search_term('not_present'))

    print(tree.similarity('Sour_Candy_bag', 'Sour_Candy_bag'))
    print(tree.similarity('Sour_Candy_bag', 'Gummy_Candy_bag'))
    print(tree.similarity('Melons', 'pasta_bowl'))
