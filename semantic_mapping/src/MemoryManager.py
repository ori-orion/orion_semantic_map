import pymongo
import pymongo.cursor
import pymongo.collection
import datetime
# import rospy

# import utils

SESSION_ID = "SESSION_NUM";
UID_ENTRY = "UID";
CROSS_REF_UID = "CRSS_REF_UID";
GLOBAL_TIME_STAMP_ENTRY = "global_timestamp";


DEBUG = True;


class MemoryManager:
    def __init__(self, root="localhost", port=27017):
        self.client = pymongo.MongoClient(root, port);
        # self.clear_db();
        self.database = self.client.database_test;

        self.collections:dict = {};

        #region Setting up the session stuff.
        session_log = self.database.session_log_coll;
        if session_log.estimated_document_count() > 0:
            #https://stackoverflow.com/questions/32076382/mongodb-how-to-get-max-value-from-collections
            # => .find().sort(...).limit(...) is actually quite efficient
            previous_session_cursor:pymongo.cursor.Cursor = session_log.find().sort(SESSION_ID, -1).limit(1);
            previous_session:list = list(previous_session_cursor);
            self.current_session_id = previous_session[0][SESSION_ID] + 1;
        else:
            self.current_session_id = 1;
        print("Session:", self.current_session_id);

        session_log.insert_one({
            SESSION_ID: self.current_session_id,
            GLOBAL_TIME_STAMP_ENTRY: datetime.datetime.now()
        });
        #endregion

        pass;

    def addCollection(self, collection_name:str) -> pymongo.collection.Collection:
        self.collections[collection_name] = self.database[collection_name];
        return self.collections[collection_name];

    def clear_db(self):
        self.client.drop_database('database_test');
        pass;