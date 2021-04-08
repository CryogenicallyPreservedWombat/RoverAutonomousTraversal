import numpy as np

class GridNode:
    """A class that represents a node in the rover's environment

    Attributes
    ----------
    coords : (int, int)
        the row and column of the grid in which the node is located
    is_obstacle : bool
        whether or not the space represented by the node contains an obstacle
    is_padding : bool
        whether or not the space represented by the node is padding around an obstacle
    on_path : bool
        whether or not the node is a point on the rover's path
    is_rover : bool
        whether or not the rover's position
    parent : GridNode
        this property is used for pathfinding. When running A*, this property helps keep track of which paths are available. Generally, all nodes on the path (except for the first one) have a parent, while all others do not.

    Methods
    -------
    __repr__()
        Returns a representation of the node as a string
    """
    def __init__(self, row, column, is_obstacle=False, is_padding=False, on_path=False, is_rover = False):
        """Initializes a new GridNode instance.

        Parameters
        ----------
        row : int
            the row on which the node is located
        column : int
            the row on which the node is located
        is_obstacle : bool
            whether or not the space represented by the node contains an obstacle
        is_padding : bool
            whether or not the space represented by the node is padding around an obstacle
        on_path : bool
            whether or not the node is a point on the rover's path
        is_rover : bool
            whether or not the rover's position

        """
        self.coords = (row, column)
        self.is_obstacle = is_obstacle
        self.is_padding = is_padding
        self.on_path = on_path
        self.is_rover = is_rover
        self.parent = None

    def __repr__(self):
        """Returns a string representing the node. The string depends on whether or not the node represents an obstacle, padding, a point on the rover's path, or the rover itself
        """
        if self.is_rover:
            return "r"
        elif self.is_obstacle:
            return "x"
        elif self.is_padding:
            return "-"
        elif self.on_path:
            return "p"
        else:
            return " "

def neighbouring_nodes(node, grid, include_diagonals=True, radius=1):
    """Finds a list of nodes around a specified node on a grid

    Parameters
    ----------
    node : GridNode
        the node whose neighbours the function finds
    grid : Grid
        the grid on which the nodes are located
    include_diagonals : bool
        whether or not nodes diagonal to the initial node should be considered potential neighbours
    radius : int
        the amount of nodes in each vertical and horizontal direction to consider as potential neighbours
    
    Returns
    -------
    A list of nodes that neighbour the specified node
    """
    node_list = []
    i, j = node.coords
    # n n n
    # n x n
    # n n n
    if include_diagonals:
        for i_offset in range(-radius, radius + 1):
            for j_offset in range(-radius, radius + 1):
                # Ensure the node does not count itself among its neighbours
                if i_offset == 0 and j_offset == 0: continue
                # Ensure the new node is within the bounds of the grid
                if i + i_offset in range(grid.height) and j + j_offset in range(grid.width):
                    node_list.append(grid[i + i_offset][j + j_offset])
    #   n
    # n x n
    #   n
    else:
        for i_offset in [dist for dist in range(-radius, radius + 1) if dist != 0]:
            if i + i_offset in range(grid.height):
                node_list.append(grid[i + i_offset][j])
        for j_offset in [dist for dist in range(-radius, radius + 1) if dist != 0]:
            if j + j_offset in range(grid.width):
                node_list.append(grid[i][j + j_offset])
    
    return node_list

def distance(node1, node2, euclidean=True):
    """Computes the distance between two nodes

    Parameters
    ----------
    node1 : GridNode
        the first node
    node2 : GridNode
        the second node
    euclidean : bool
        whether or not to use euclidean distance. If true, the square of the euclidean distance is used (as it is computed slightly more efficiently). If false, Manhattan distance is used
    
    Returns
    -------
    A float that represents the distance between the two nodes
    """
    x1, y1 = node1.coords
    x2, y2 = node2.coords
    if euclidean:
        return (x2 - x1) ** 2 + (y2 - y1) ** 2
    else:
        return abs(x2 - x1) + abs(y2 - y1)
