from grid_node import distance, neighbouring_nodes
import numpy as np

def quickest_path(node1, node2, grid, include_diagonals=True, euclidean=True, verbose=False):
    """Finds the shortest path between two nodes on a grid, avoiding obstacles

    Parameters
    ----------
    node1 : GridNode
        The node that represents the beginning of the path
    node2 : GridNode
        The node that represents the desired destination
    grid : Grid
        A grid that contains `node1`, `node2`, and a representation of the rover's environment

    Returns
    -------
    list
        A list of nodes that forms the optimal path
        
    Rasies
    -----
    AssertionError
        If no viable path is found
    """

    for node_array in grid._array:
        for node in node_array:
            node.parents = []

    open_set = [node1]
    closed_set = []
    start_dist = { node1: 0 }
    
    while len(open_set) != 0:
        # Casting the map to a list is redundant in Python 2.x, but is necessary in Python 3.x
        h_values = list(map(
                lambda n : distance(n, node2, euclidean=euclidean) + start_dist[n],
                 open_set))

        index = np.argmin(np.array(h_values))

        current_node = open_set.pop(index)

        if current_node is node2:
            if verbose:
                print("Found a valid path")
            break 

        for node in neighbouring_nodes(current_node, grid, include_diagonals=include_diagonals):

            # Don't consider obstacles among valid routes
            if node.is_obstacle or node.is_padding:
                continue

            g_value = start_dist[current_node] + grid.node_spacing
            if node in open_set:
                if start_dist[node] <= g_value:
                    continue
            elif node in closed_set:
                if start_dist[node] <= g_value: continue
                closed_set.remove(node)
                open_set.append(node)
            else:
                open_set.append(node)
                start_dist[node] = g_value
            node.parents = [current_node]

        closed_set.append(current_node)
    
    # Recover path from grid
    node = node2
    path = []


    while len(node.parents) > 0:
        path.append(node)
        node = node.parents[0]
    
    assert node is node1, ("Viable path was not found\n" + repr(grid))
    return list(reversed(path))

def tuple_difference(tuple1, tuple2):
    return (tuple1[0] - tuple2[0], tuple1[1] - tuple2[1])

def stitch_colinear_nodes(start_node, path):
    if len(path) <= 1: return path
    
    diff = tuple_difference(path[0].coords, start_node.coords)
    
    for i in range(1, len(path)):
        if tuple_difference(path[i].coords, path[i - 1].coords) != diff:
            return [path[i - 1]] + stitch_colinear_nodes(path[i - 1], path[i:])
        print("Stitching node at {} with node {}".format(path[i].coords, path[i - 1].coords))