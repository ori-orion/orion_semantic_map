"""
Author: Matthew Munks
Owner: Matthew Munks
"""

from typing import Any, Dict
import pymongo
import pymongo.cursor
import pymongo.collection
import datetime
import rospy

import std_srvs.srv

from new_version.constants import SERVICE_ROOT, SESSION_KEY

GLOBAL_TIME_STAMP_ENTRY = "global_timestamp"


class MemoryManager:
    """
    The manager for connecting to the pymongo database.
    """

    def __init__(
        self, root="localhost", port=62345, connect_to_current_if_available=False
    ):
        """
        Most of these are self explanatory - the pymongo database hosts itself in localhost on
        a given port.

        connect_to_current_if_available - Let's say there are multiple memory systems (or instances of
            this class) running on the robot at a given time. If this flag is True, then it will
            latch onto the previous session (hopefully the one started in the other rosnode) rather
            than creating a new one. That does however imply a start up order for the rosnodes.
        """
        self.client = pymongo.MongoClient(root, port)

        self.database = self.client.robocup

        self.collections: dict = {}

        self.setSessionNumber(connect_to_current_if_available)

        self.setupServices()

    def setSessionNumber(self, connect_to_current_if_available: bool):
        session_log_collection = self.database.session_logs
        if session_log_collection.estimated_document_count() > 0:
            # https://stackoverflow.com/questions/32076382/mongodb-how-to-get-max-value-from-collections
            # => .find().sort(...).limit(...) is actually quite efficient
            previous_session_cursor: pymongo.cursor.Cursor = (
                session_log_collection.find()
                .sort(SESSION_KEY, pymongo.DESCENDING)
                .limit(1)
            )
            previous_session: Dict[str, Any] = previous_session_cursor.next()
            if connect_to_current_if_available:
                self.current_session_num = previous_session[SESSION_KEY]
            else:
                self.current_session_num = previous_session[SESSION_KEY] + 1
        else:
            self.current_session_num = 1

        print("Session:", self.current_session_num)

        if self.current_session_num == 1 or not connect_to_current_if_available:
            session_log_collection.insert_one(
                {
                    SESSION_KEY: self.current_session_num,
                    GLOBAL_TIME_STAMP_ENTRY: datetime.datetime.now(),
                }
            )

    def getCollection(self, collection_name: str) -> pymongo.collection.Collection:
        """
        Get reference to the collection from the database.
        If it is a new collection, it will be added to the database.
        """
        return self.database[collection_name]

    def clear_db(self):
        """
        Gets rid of the entire database.
        """
        self.client.drop_database(self.database.name)

    def clearDatabaseROSEntrypoint(self, _: std_srvs.srv.EmptyRequest):
        """
        ROS entrypoint for deleting the entire database.
        """
        self.clear_db()
        return std_srvs.srv.EmptyResponse()

    def setupServices(self):
        """
        Function to setup all the services.
        """
        rospy.Service(
            SERVICE_ROOT + "delete_databases",
            std_srvs.srv.Empty,
            self.clearDatabaseROSEntrypoint,
        )
