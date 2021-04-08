from grid_node import GridNode
import numpy as np
from math import ceil

class Grid:
    """
    A class that provides a discrete representation of 2D space to facilitate pathfinding in a continuous environment.

    Attributes
    ----------
    start : (float, float)
        a point that defines the origin of the grid
    end : (float, float)
        a point that defines the corner of the grid opposite `start`
    node_spacing : float
        the length of each grid square in meters
    width : int
        the number of nodes in each row; equivalently the number of columns in the grid 
    height : int
        the number of nodes in each column; equivalently the number of rows in the grid
    _array : nparray
        the underlying numpy array in which the contents of the grid is stored

    Methods
    -------
    @staticmethod
    oversized_grid(start, end, node_spacing=1.0, buffer_distance=5.0)
        Creates an instance of grid larger than made necessary by the start and end points
    nearest_node(self, point)
        Finds the node whose physical location is closest to the specified point 
    location(self, row, column)
        Finds the physical position of a node at the specified indices 
    __getitem__(indices)
        Returns the item in the underlying array located at the specified indices
    __setitem__(indices, value)
        Sets the item in the underlying array located at the specified indices to the given value
    __repr__()
        Returns a representation of the grid as a string
    """

    def __init__(self, start, end, node_spacing=1.0):
        """Initializes a new Grid instance.

        Parameters
        ----------
        start : (float, float)
            a point that defines the origin of the grid
        end : (float, float)
            a point that defines the corner of the grid opposite `start`
        node_spacing : float
            the length of each grid square in meters
        """
        
        self.start = start
        self.end = end

        pre_array = []
        num_x_boxes = int(ceil(abs(float(end[0] - start[0]) / node_spacing)))
        num_y_boxes = int(ceil(abs(float(end[1] - start[1]) / node_spacing)))

        # Ensures the array is not empty
        num_x_boxes = max(num_x_boxes, 1)
        num_y_boxes = max(num_y_boxes, 1)

        for i in range(num_y_boxes):
            for j in range(num_x_boxes):
                pre_array.append(GridNode(i, j))
        
        self._array = np.array(pre_array).reshape((num_y_boxes, num_x_boxes))
        self.width = num_x_boxes
        self.height = num_y_boxes
        self.node_spacing = node_spacing

    def __getitem__(self, indices):
        """Allows a value to be read from a grid through subscripts"""
        return self._array[indices]
    
    def __setitem__(self, indices, value):
        """Allows a value to be set on an element of a grid through subscripts"""
        self._array[indices] = value
    
    def __repr__(self):
        """Provides a string representation of an instance of Grid"""
        string = "["

        for i in range(self.height):
            for j in range(self.width):
                node = self[i][j]
                # Ensures there are no trailing spaces or newlines
                string += repr(node) + ("]" if j == self.width - 1 else " ")
            string += ("" if i == self.height - 1 else "\n[")
        
        return string
    
    def nearest_node(self, point):
        """Finds the node whose physical location is closest to the specified point

        Parameters
        ----------
        point : (float, float)
            a point to be approximated by the location of a node

        Returns
        -------
        The node on the grid nearest to the point 

        """
        # Unnecessary float casts are for backwards compatibility to Python 2.7
        # Finds the element of the Grid that is nearest to a specified point
        y_distance = float(self.end[1] - self.start[1])
        x_distance = float(self.end[0] - self.start[0])

        i = int(round(self.height * (point[1] - self.start[1]) / y_distance)) if y_distance != 0 else 0
        j = int(round(self.width * (point[0] - self.start[0]) / x_distance)) if x_distance != 0 else 0

        # Ensures values are within bounds
        i = min(i, self.height - 1)
        i = max(i, 0)

        j = min(j, self.width - 1)
        j = max(j, 0)
        
        return self[i][j]
    
    def location(self, row, column):
        """Finds the physical position of a node at the specified indices

        Parameters
        ----------
        row : int
            the row of the node whose physical location is sought
        column : int
            the column of the node whose physical location is sought

        Returns
        -------
        A tuple containing the physical location of the grid 

        """
        # Finds the coordinates of a node on the grid, given the indices that describe its position
        x = self.start[0] + float(column) / self.width * (self.end[0] - self.start[0])
        y = self.start[1] + float(row) / self.height * (self.end[1] - self.start[1])
        return (x, y)
    
    @staticmethod
    def oversized_grid(start, end, node_spacing=1.0, buffer_distance=5.0):
        """Creates an instance of grid larger than made necessary by the start and end points

        Parameters
        ----------
        start : (float, float)
            a point that defines the origin of the grid
        end : (float, float)
            a point that defines the corner of the grid opposite `start`
        node_spacing : float
            the length of each grid square in meters
        buffer_distance : float
            the amount of distance in each direction by which the grid should be extended beyond what is necessary to fit both the rover and its destination

        Returns
        -------
        A new instance of Grid

        """


        if buffer_distance < 0:
            return Grid.oversized_grid(start, end, node_spacing=node_spacing, buffer_distance=-buffer_distance)

        x_sign = 1 if end[0] >= start[0] else -1
        y_sign = 1 if end[1] >= start[1] else -1

        new_start = (start[0] - buffer_distance * x_sign, start[1] - buffer_distance * y_sign)
        new_end = (end[0] + buffer_distance * x_sign, end[1] + buffer_distance * y_sign)

        return Grid(new_start, new_end, node_spacing=node_spacing)

