from CollectionManager import CollectionManager, TypesCollection;


class ConsistencyChecker(CollectionManager):
    def __init__(self, based_off:CollectionManager, types:TypesCollection, service_name:str):
        super(ConsistencyChecker, self).__init__(
            type=types, 
            service_name=service_name, 
            memory_manager=based_off.memory_manager);

        self.based_off = based_off;

        

    pass;