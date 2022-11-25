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
        Load a `OntologyTree` from a file. The file should have a term for each line, written in
        the form <parent>/<term>. If the parent has not yet been added, it will be added directly 
        to the root of the tree. For example:
        
        food/fruit
        fruit/apple
        food/cookie
        
        will create a tree with branches: 
            <base_label> -> food -> fruit -> apple  
            <base_label> -> food -> cookie
        """
        tree = cls(base_label)

        with open(filename) as input_file:
            for line in input_file.readlines():
                cleaned_line = line.replace('\n', '') # Remove line endings
                parent, child = cleaned_line.split('/')
                tree.add_term(parent, child)

        return tree

    def add_term(self, parent: str, child: str):
        """
        Add the `child` term to the tree as child of `parent`. If the parent is not yet in the 
        tree, it will be added as child of the root node.
        """
        hierarchy = self.search_term(parent)

        categories = [parent]
        if hierarchy:
            categories = hierarchy[1:] # Exclude the base label
        
        categories += [child]
        self.root.add_term(categories)

    
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

    def print_tree(self):
        self.root.print_graph()


if __name__ == '__main__':
    tree = OntologyTree.from_file(os.path.dirname(__file__) + '/taxonomyLabels.txt')
    
    print(tree.search_term('Sour_Candy_bag'))
    print(tree.search_term('not_present'))

    print(tree.similarity('Sour_Candy_bag', 'Sour_Candy_bag'))
    print(tree.similarity('Sour_Candy_bag', 'Gummy_Candy_bag'))
    print(tree.similarity('Melons', 'pasta_bowl'))

    tree.print_tree()
