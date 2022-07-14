"""
Author: Matthew Munks
Owner: Matthew Munks
"""

import pymongo
import pymongo.cursor
import pymongo.collection
import datetime
import rospy

import std_srvs.srv;

# import utils
from utils import UID_ENTRY, SESSION_ID
GLOBAL_TIME_STAMP_ENTRY = "global_timestamp";


DEBUG = True;
DEBUG_LONG = False;

# The root for all things som related.
SERVICE_ROOT = "som/";


class MemoryManager:
    def __init__(self, root="localhost", port=62345):
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

        self.setup_services();

    def addCollection(self, collection_name:str) -> pymongo.collection.Collection:
        self.collections[collection_name] = self.database[collection_name];
        return self.collections[collection_name];

    def clear_db(self):
        self.client.drop_database('database_test');

    def clear_database_ROS_server(self, srv_input:std_srvs.srv.EmptyRequest):
        self.clear_db();
        return std_srvs.srv.EmptyResponse();

    def setup_services(self):
        rospy.Service(SERVICE_ROOT + "delete_databases", std_srvs.srv.Empty, self.clear_database_ROS_server);