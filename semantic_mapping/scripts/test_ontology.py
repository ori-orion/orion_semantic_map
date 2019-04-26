#!/usr/bin/env python

from ontology import Ontology


my_onto = Ontology()

print(my_onto.check_class_exists("Food"))
print(my_onto.check_class_exists("Not food"))
print(my_onto.get_children("Food"))
