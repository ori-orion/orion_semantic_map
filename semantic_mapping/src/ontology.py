import ontospy
import os

class Ontology():
    """
    contains definition and convenience methods for semantic ontology
    """
    def __init__(self, filename = "robocupontology.owl"):
        dir = os.path.dirname(__file__)
        file_dir = os.path.join(dir, '../config/', filename)
        self.ontology = ontospy.Ontospy(file_dir)

    def check_class_exists(self, label):
        """
        Argument is class string label. Returns true if class exists in ontology
        and false otherwise
        """
        exists = False
        for onto_class in self.ontology.all_classes:
            if onto_class.bestLabel().toPython() == label:
                exists = True
        return exists

    def get_children(self, label):
        """
        Returns list of labels of children of object, including the label
        of the object itself.
        """
        if not self.check_class_exists(label):
            raise Exception("This object type does not exist in ontology")
            return []

        for onto_class in self.ontology.all_classes:
            if onto_class.bestLabel().toPython() == label:
                list_of_children = [str(onto_class.bestLabel().toPython())]
                children = onto_class.children()
                break

        for child in children:
            list_of_children.append(str(child.bestLabel().toPython()))

        return list_of_children
