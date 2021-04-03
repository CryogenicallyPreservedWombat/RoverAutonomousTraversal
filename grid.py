from grid_node import GridNode
import numpy as np
from math import ceil

class Grid:
    """
    A class that provides a discrete representation of 2D space to facilitate pathfinding in a continuous environment.
    """

    def __init__(self, start, end, side_length=1.0):
        """
        Initializes a new Grid instance.

        Parameters
        ----------
        start : (float, float)
            a point that defines the origin of the grid
        end : (float, float)
            a point that defines the corner of the grid opposite `start`
        side_length : float
            the length of each grid square in meters
        """
        
        self.start = start
        self.end = end

        pre_array = []
        num_x_boxes = int(ceil(abs(float(end[0] - start[0]) / side_length)))
        num_y_boxes = int(ceil(abs(float(end[1] - start[1]) / side_length)))

        # Ensures the array is not empty
        num_x_boxes = max(num_x_boxes, 1)
        num_y_boxes = max(num_y_boxes, 1)

        for i in range(num_y_boxes):
            for j in range(num_x_boxes):
                pre_array.append(GridNode(i, j))
        
        self._array = np.array(pre_array).reshape((num_y_boxes, num_x_boxes))
        self.width = num_x_boxes
        self.height = num_y_boxes
        self.side_length = side_length

    def __getitem__(self, indices):
        # Allows a value to be read from an instace of Grid through a subscript
        return self._array[indices]
    
    def __setitem__(self, indices, value):
        # Allows a value to be set from an instace of Grid through a subscript
        self._array[indices] = value
    
    def __repr__(self):
        # Provides a string representation of an instance of Grid
        string = ""

        for i in range(self.height):
            for j in range(self.width):
                node = self[i][j]
                # Ensures there are no trailing spaces or newlines
                string += repr(node) + ("]" if j == self.width - 1 else " ")
            string += ("" if i == self.height - 1 else "\n[")
        
        return string
    
    def nearest_node(self, point):
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
        # Finds the coordinates of a node on the grid, given the indices that describe its position
        x = self.start[0] + float(column) / self.width * (self.end[0] - self.start[0])
        y = self.start[1] + float(row) / self.height * (self.end[1] - self.start[1])
        return (x, y)
