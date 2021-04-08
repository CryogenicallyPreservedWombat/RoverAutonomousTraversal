from grid_node import distance, neighbouring_nodes
import numpy as np
from errors import NoValidPathError

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
    ------
    NoValidPathError
        If no viable path is found
    """

    for node_array in grid._array:
        for node in node_array:
            node.parent = None

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
            node.parent = current_node

        closed_set.append(current_node)
    
    # Recover path from grid
    node = node2
    path = []


    while node.parent is not None:
        path.append(node)
        node = node.parent
    
    if node is not node1:
        print("Viable path was not found\n" + repr(grid))
        raise NoValidPathError
    
    return list(reversed(path))

def _tuple_difference(tuple1, tuple2):
    """Element-wise subtraction of two, two-dimensional tuples

    Parameters
    ----------
    tuple1 : tuple (2 elements)
        The minuend, from which tuple2 is subtracted
    tuple2 : tuple (2 elements)
        The subtrahend, which is subtracted from tuple1
    
    Returns
    -------
    The difference between tuple1 and tuple2
    """
    return (tuple1[0] - tuple2[0], tuple1[1] - tuple2[1])

def stitch_colinear_nodes(start_node, path):
    """Simplifies a path by removing nodes that fall on a line through two other nodes

    A path fed into the function might look like:
    p p p p p
              p p p
    
    While the stitched path would instead be:
    p _ _ _ _ p
              p _ p
    
    This saves time during rover movement

    Parameters
    ----------
    start_node : GridNode
        The node from which the rover is assumed to begin. Note that this is distinct from the first node of the path, because the path is defined to be a series of nodes the rover has yet to cover and does not include the node on which the rover itself is located
    path : list of GridNode
        The list of nodes that defines the rover's path
    
    Returns
    -------
    A list of nodes, containing only the nodes necessary to define the original path
    """
    if len(path) <= 1: 
        return path
    
    diff = _tuple_difference(path[0].coords, start_node.coords)
    
    for i in range(1, len(path)):
        counter += 1
        if _tuple_difference(path[i].coords, path[i - 1].coords) != diff:
            return [path[i - 1]] + stitch_colinear_nodes(path[i - 1], path[i:])
    return [path[-1]]
