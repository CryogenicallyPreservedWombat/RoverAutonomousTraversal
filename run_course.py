from pathfinding import quickest_path, stitch_colinear_nodes
from move_rover import move_rover
from locate_obstacles import locate_obstacles
from grid_node import distance, neighbouring_nodes
from grid import Grid
from math import ceil
from errors import ObseleteGridError, NoValidPathError

def run_course(rover, end_point, node_spacing=0.4, include_diagonals=True, euclidean=True, verbose=False, sensors_to_ignore=[7], obstacle_padding=0.4, buffer_distance=5.0):
    """Navigates the rover from its current location to a specified endpoint

    Parameters
    ----------
    rover : Rover
        the rover to be navigated through the obstacles
    end_point : tuple
        the coordinates of the rover's intended destination
    node_spacing : float
        the distance between nodes on the grid. This affects the size and accuracy of the grid, both of which increase as the node_spacing increases
    include_diagonals : bool
        whether or not the rover should be able to move diagonally between nodes
    euclidean : bool
        whether or not euclidean distance should be used as the heuristic function when running A*. If not, Manhattan distance is used
    verbose : bool
        whether or not to include verbose logging during the running of the function
    sensors_to_ignore : list of ints
        indices of LiDAR sensors to be ignored by the rover. This is set to [7] by default, as the Gazebo simulation of the QSET picks up erroneous readings from that specific sensor
    obstacle_padding : float
        the amount of distance from each obstacle the rover should maintain. The higher this value, the higher the likelihood of the rover seeing no valid paths
    buffer_distance : float
        the amount of distance in each direction by which the grid should be extended beyond what is necessary to fit both the rover and its destination
    """
    recalculate_route = False
    start_point = (rover.x, rover.y)

    # Grid is oversized in case obstacles force the rover's path away from a straight line
    grid = Grid.oversized_grid(start_point, end_point, node_spacing=node_spacing, buffer_distance=buffer_distance)

    start_node = grid.nearest_node(start_point)
    end_node = grid.nearest_node(end_point)

    path = quickest_path(start_node, end_node, grid, include_diagonals=include_diagonals, euclidean=euclidean)
    stitched_path = stitch_colinear_nodes(start_node, path)
    force_loop_run = False

    while len(stitched_path) != 0 or force_loop_run:

        force_loop_run = False
        obstacles = locate_obstacles(rover, sensors_to_ignore=sensors_to_ignore)

        for obstacle in obstacles:
            obstacle_node = grid.nearest_node(obstacle)

            if not obstacle_node.is_obstacle:

                obstacle_node.is_obstacle = True
                recalculate_route = True

                # Calculates the appropriate amount of padding to give each obstacle
                padding_layers = int(ceil(float(obstacle_padding) / grid.node_spacing))

                for node in neighbouring_nodes(obstacle_node, grid, include_diagonals=include_diagonals, radius=padding_layers):
                    if node is not grid.nearest_node((rover.x, rover.y)):
                        node.is_padding = True


        if recalculate_route:
            if verbose:
                print("Found new obstacles, recalculating route")
            # might want to create an entirely new grid based on the current rover's position
            rover_node = grid.nearest_node((rover.x, rover.y))
            try:
                path = quickest_path(rover_node, end_node, grid, include_diagonals=include_diagonals, euclidean=euclidean, verbose=verbose)
            except:
                if verbose: print("ERROR: NO ROUTE FOUND.\nREDRAWING GRID")
                for row in grid._array:
                    for node in row:
                        node.is_padding = False
                        node.is_obstacle = False
                continue

            stitched_path = stitch_colinear_nodes(rover_node, path)
            recalculate_route = False
        
        if verbose:
            for row in grid._array:
                for node in row:
                    node.is_rover = grid.nearest_node((rover.x, rover.y)) is node
                    node.on_path = node in stitched_path
            print(grid)

        next_node = stitched_path.pop(0)
        row, column = next_node.coords
        next_location = grid.location(row, column)
        if verbose:
            print("Moving to location {} from {}".format((round(next_location[0], 2), round(next_location[1], 2)), (round(rover.x, 2), round(rover.y, 2))))

        try:
            move_rover(rover, next_location[0], next_location[1], grid)
        except:
            if verbose: print("Encountered an obstacle while turning, recalculating route")
            recalculate_route = True
            force_loop_run = True
            continue

        if verbose:
            print("Move completed, currently predicting {} more waypoints".format(len(stitched_path)))
    
    rover.send_command(0, 0)