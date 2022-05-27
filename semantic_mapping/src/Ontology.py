"""
Constructs the ontology tree from labels.txt.
"""

class ontology_member:
    def __init__(self, label:str="Objs", children:list=[]):
        self.label:str = label;
        self.children:list = children;

    def search_for_term(self, term:str) -> list:
        if (term == self.label):
            return [self.label];

        for element in self.children:
            element:ontology_member;
            output = element.search_for_term(term);
            if output != None:
                output.append(self.label);
                return output;

        return None;
    
    def print_graph(self, num_tabs=0):
        if (num_tabs == 0):
            prefix = "";
        else:
            prefix = "    "*(num_tabs-1) + " |->";
        
        print(prefix, self.label);
        for element in self.children:
            element:ontology_member;
            element.print_graph(num_tabs=num_tabs+1);

def read_file(filename="labels.txt"):
    
    pass;

