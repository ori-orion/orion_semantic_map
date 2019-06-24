import ontospy
import os

class Ontology():
    """
    contains definition and convenience methods for semantic ontology
    """
    def __init__(self, filename):
        dir = os.path.dirname(__file__)
        file_dir = os.path.join(dir, '../../config/', filename)
        self.ontology = ontospy.Ontospy(file_dir)

    def check_class_exists(self, label):
        """
        Argument is class string label. Returns true if class exists in ontology
        and false otherwise
        """
        exists = False
        for onto_class in self.ontology.all_classes:
            if onto_class.bestLabel().toPython().lower() == label.lower():
                exists = True
        return exists

    def get_valid_types(self, label):
        """
        Returns list of labels of children of object, including the label
        of the object itself.
        """
        if not self.check_class_exists(label):
            raise Exception("This object type does not exist in ontology")
            return []

        for onto_class in self.ontology.all_classes:
            if str(onto_class.bestLabel().toPython()).lower() == label.lower():
                list_of_children = [str(onto_class.bestLabel().toPython()).lower()]
                children = onto_class.children()
                break

        for child in children:
            list_of_children.append(str(child.bestLabel().toPython()).lower())

        return list_of_children


    def get_object_tree(self, label):
        """
        Returns the ontology tree of an object as a list.

        Consider this ontology:
        -> thing
           -> food
               -> fruit
                   -> banana
                   -> apple

        get_object_tree("banana") returns ["banana", "fruit", "food", "thing"]
        """

    def check_similarity(self, obs1, obs2):
        """
        Returns the similarity of the object type of two som observations. Pairs of object types with lower similarity are more similar.

        Similarity integer represents the average number of levels to a common ancestor of the objects. For example, consider the ontology:
         -> thing
            -> food
                -> fruit
                    -> banana
                    -> apple
                -> cereal
            -> drink
                -> coke
                -> milk
         (banana, apple) returns 1.0 as both 1 level to common ancestor "fruit"
         (fruit, coke) returns 2.0 as both 2 levels to common ancester "thing"
         (banana, banana) returns 0.0
         (cereal, banana) returns 1.5, the average of "cereal" 1 level from "food" and "banana", two levels from "food"
         """
        if (not self.check_class_exists(obs1.type)) or (not self.check_class_exists(obs2.type)):
            raise Exception('Type specified in observation is not valid for similarity check. Valid object types are those in the ontology.')
            return float("inf")

        parent_tree1 = self.get_parent_tree(obs1.type)
        parent_tree2 = self.get_parent_tree(obs2.type)

        for type in parent_tree1:
            if type in parent_tree2:
                common_type = type
                break
        index1 = parent_tree1.index(common_type)
        index2 = parent_tree2.index(common_type)
        return (float(index1 + index2)/2.0), common_type


    def get_parent_tree(self, label):
        for onto_class in self.ontology.all_classes:
            if onto_class.bestLabel().toPython().lower() == label.lower():
                break

        parent_tree = [str(onto_class.bestLabel().toPython().lower())]
        while len(onto_class.parents()) > 0:
            onto_class = onto_class.parents()[0]
            parent_tree.append(str(onto_class.bestLabel().toPython().lower()))
        parent_tree.append('thing')
        return parent_tree
