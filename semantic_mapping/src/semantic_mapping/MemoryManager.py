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
    """
    The manager for the connecting to the pymongo database.
    """
    def __init__(self, root="localhost", port=62345, connect_to_current_latest=False):
        """
        Most of these are self explanatory - the pymongo database hosts itself in localhost on 
        a given port.

        connect_to_current_latest - Let's say there are multiple memory systems (or instances of
            this class) running on the robot at a given time. If this flag is True, then it will
            latch onto the previous session (hopefully the one started in the other rosnode) rather 
            than creating a new one. That does however imply a start up order for the rosnodes.
        """
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
            if connect_to_current_latest == True:
                self.current_session_id = previous_session[0][SESSION_ID];
            else:
                self.current_session_id = previous_session[0][SESSION_ID] + 1;
        else:
            self.current_session_id = 1;
        print("Session:", self.current_session_id);

        if self.current_session_id == 1 or connect_to_current_latest==False:
            session_log.insert_one({
                SESSION_ID: self.current_session_id,
                GLOBAL_TIME_STAMP_ENTRY: datetime.datetime.now()
            });
        #endregion

        self.setup_services();

    def addCollection(self, collection_name:str) -> pymongo.collection.Collection:
        """
        Adds a collection to the database.
        If the collection already exists, this acts as a get funciton.
        """
        self.collections[collection_name] = self.database[collection_name];
        return self.collections[collection_name];

    def clear_db(self, database_name='database_test'):
        """
        Gets rid of the entire database.
        """
        self.client.drop_database(database_name);

    def clear_database_ROS_server(self, srv_input:std_srvs.srv.EmptyRequest):
        """
        ROS entrypoint for deleting the entire database.
        """
        self.clear_db();
        return std_srvs.srv.EmptyResponse();

    def setup_services(self):
        """
        Function to setup all the services.
        """
        rospy.Service(SERVICE_ROOT + "delete_databases", std_srvs.srv.Empty, self.clear_database_ROS_server);