"""
Defines constants to be used in SOM.
"""



import enum



class HumanPose(enum.Enum):
    """An enum of possible human poses that can be registered."""
    STANDING = 1
    SITTING = 2
    LAYING_DOWN = 3



class Relation():
    """Specifies relationships between two objects."""
    left = False    # One item is to the left of another (w.r.t. robot)
    right = False   # One item is to the right of another (w.r.t. robot)
    above = False   # One item is above another if on shelf above (w.r.t. robot)
    below = False   # One item is below another if on shelf below (w.r.t. robot)
    at = False      # A person can be at a table/node
    ontop = False   # An item can be ontop of another (e.g. table)
    behind = False
    frontof = False



class RoomType(enum.Enum):
    """A list of possible room types that we can have"""
    KITCHEN = 1
    LIVING_ROOM = 2
    STUDY = 3
    BATHROOM = 4
    TOILET = 5
    HALLWAY = 6
