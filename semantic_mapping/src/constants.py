"""
Defines constants to be used in SOM.
"""



import enum



class HumanPose(enum.Enum):
    """An enum of possible human poses that can be registered."""
    STANDING = 1
    SITTING = 2
    LAYING_DOWN = 3


class RoomType(enum.Enum):
    """A list of possible room types that we can have"""
    KITCHEN = 1
    LIVING_ROOM = 2
    STUDY = 3
    BATHROOM = 4
    TOILET = 5
    HALLWAY = 6
