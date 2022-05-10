from numpy import void
import pymongo
import pymongo.cursor
import datetime
import rospy

import utils

SESSION_ID = "session_num";
UID_ENTRY = "entry_uid";
GLOBAL_TIME_STAMP_ENTRY = "global_timestamp";

class MemoryManager:
    def __init__(self, root="localhost", port=62345):
        self.client = pymongo.MongoClient(root, port);
        self.database = self.client.database;

        self.collections = {};

        #region Setting up the session stuff.
        session_log = self.main_database.session_log_coll;
        print(session_log.estimated_document_count());
        if session_log.estimated_document_count() > 0:
            #https://stackoverflow.com/questions/32076382/mongodb-how-to-get-max-value-from-collections
            # => .find().sort(...).limit(...) is actually quite efficient
            previous_session_cursor:pymongo.cursor.Cursor = session_log.find().sort(SESSION_ID, -1).limit(1);
            previous_session:list = list(previous_session_cursor);
            self.current_session_id = previous_session[0][SESSION_ID] + 1;
        else:
            self.current_session_id = 1;

        session_log.insert_one({
            SESSION_ID: self.current_session_id,
            GLOBAL_TIME_STAMP_ENTRY: datetime.datetime.now()
        });
        #endregion

        pass;

    def addCollection(self, collection_name:str) -> void:
        self.collections[collection_name] = self.database[collection_name];
        pass;