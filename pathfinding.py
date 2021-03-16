from grid_node import distance, neighbouring_nodes
import numpy as np

def quickest_path(node1, node2, grid, include_diagonals=True, euclidean=True):

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
            break 

        for node in neighbouring_nodes(current_node, grid, include_diagonals=include_diagonals):

            # Don't consider obstacles among valid routes
            if node.is_obstacle:
                continue

            g_value = start_dist[current_node] + 1
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
    
    assert node is grid[0][0], "Viable path was not found"
    return list(reversed(path))
