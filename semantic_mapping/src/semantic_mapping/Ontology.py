"""
Constructs the ontology tree from labels.txt.

Author: Matthew Munks
Owner: Matthew Munks
"""

class ontology_member:
    def __init__(self, label:str="Objs"):
        self.label:str = label;
        self.children:list = [];

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
            prefix = "|   "*(num_tabs-1) + "|->";
        
        print(prefix, self.label);
        for element in self.children:
            element:ontology_member;
            element.print_graph(num_tabs=num_tabs+1);

    def add_term(self, adding:list):
        # Here the first member of the list is the label of the child of the
        # current member. The second is that of the child's child. etc.

        if len(adding) == 0:
            return;

        for element in self.children:
            element:ontology_member;
            if element.label == adding[0]:
                element.add_term(adding[1:]);
                return;

        # print("Adding a new element: category:", self.label, " type: ", adding[0]," len(children)=", len(self.children));
        # Not currently in the tree;
        appending = ontology_member(label=adding[0]);
        if len(adding) > 1:
            appending.add_term(adding[1:]);
        self.children.append(appending);
        
        # print(adding[1:]);

        

def read_file(filename="./labels.txt") -> ontology_member:
    file = open(filename);
    
    ontology_root = ontology_member();

    while True:
        line = file.readline().replace('\n','');
        if not line:
            break;

        ontology_root.add_term(line.lower().split('/'));

    file.close();

    return ontology_root;

if __name__ == '__main__':
    o = read_file();

    input();

    print(o.label);
    print(o.children[0].label);
    print(len(o.children[0].children));
    print(o.children[0].children[0].label);
    print(len(o.children[0].children[0].children));

    input();
    o.print_graph();