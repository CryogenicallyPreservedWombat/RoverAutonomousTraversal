import numpy as np

class GridNode:
    def __init__(self, row, column, is_obstacle=False, is_padding=False, on_path=False, is_rover = False):
        self.coords = (row, column)
        self.is_obstacle = is_obstacle
        self.is_padding = is_padding
        self.on_path = on_path
        self.is_rover = is_rover
        self.children = []
        self.parents = []

    def __repr__(self):
        if self.is_rover:
            return "r"
        elif self.is_obstacle:
            return "x"
        elif self.is_padding:
            return "o"
        elif self.on_path:
            return "p"
        else:
            return " "

def neighbouring_nodes(node, grid, include_diagonals=True, radius=1):

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
    x1, y1 = node1.coords
    x2, y2 = node2.coords
    if euclidean:
        return (x2 - x1) ** 2 + (y2 - y1) ** 2
    else:
        return abs(x2 - x1) + abs(y2 - y1)
