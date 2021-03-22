import numpy as np

class GridNode:
    def __init__(self, row, column, is_obstacle=False, on_path=False, is_rover = False):
        self.coords = (row, column)
        self.is_obstacle = is_obstacle
        self.on_path = on_path
        self.is_rover = is_rover
        self.children = []
        self.parents = []

    def __repr__(self):
        char = "x" if self.is_obstacle else " "
        char2 = "p" if self.on_path else char
        return "r" if self.is_rover else char2

def neighbouring_nodes(node, grid, include_diagonals=True):

    node_list = []
    i, j = node.coords
    # n n n
    # n x n
    # n n n
    if include_diagonals:
        for i_offset in range(-1, 2):
            for j_offset in range(-1, 2):
                # Ensure the node does not count itself among its neighbours
                if i_offset == 0 and j_offset == 0: continue
                # Ensure the new node is within the bounds of the grid
                if i + i_offset in range(grid.height) and j + j_offset in range(grid.width):
                    node_list.append(grid[i + i_offset][j + j_offset])
    #   n
    # n x n
    #   n
    else:
        for i_offset in (-1, 1):
            if i + i_offset in range(grid.height):
                node_list.append(grid[i + i_offset][j])
        for j_offset in (-1, 1):
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
