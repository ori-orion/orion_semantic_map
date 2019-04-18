"""
Defines constants to be used in SOM.
"""



import enum



class HumanPose(enum.Enum):
    """An enum of possible human poses that can be registered."""
    STANDING = 1
    SITTING = 2
    LAYING_DOWN = 3



class Relation(enum.Enum):
    """An enum of relations between objects."""
    LEFT = 1    # One item is to the left of another (w.r.t. robot)
    RIGHT = 2   # One item is to the right of another (w.r.t. robot)
    ABOVE = 3   # One item is above another if on shelf above (w.r.t. robot)
    BELOW = 4   # One item is below another if on shelf below (w.r.t. robot)
    AT = 5      # A person can be at a table/node
    ONTOP = 6   # An item can be ontop of another (e.g. table)



class RoomType(enum.Enum):
    """A list of possible room types that we can have"""
    KITCHEN = 1
    LIVING_ROOM = 2
    STUDY = 3
    BATHROOM = 4
    TOILET = 5
    HALLWAY = 6