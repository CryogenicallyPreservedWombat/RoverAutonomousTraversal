class NoValidPathError(Exception):
    """Called when the rover cannot find any valid paths between its location and its destination
    """
    pass

class ObseleteGridError(Exception):
    """Called when the rover detects new obstacles that are not recorded on the grid
    """
    pass

