from pathfinding import quickest_path, stitch_colinear_nodes
from move_rover import move_rover
from locate_obstacles import locate_obstacles
from grid_node import distance, neighbouring_nodes
from grid import Grid
from math import ceil
from errors import ObseleteGridError, NoValidPathError

def run_course(rover, end_point, node_spacing=0.4, include_diagonals=True, euclidean=True, verbose=False, sensors_to_ignore=[7], obstacle_padding=0.4, buffer_distance=5.0):
    
    recalculate_route = False
    start_point = (rover.x, rover.y)

    # Ensures the stitched paths don't extend beyond LiDAR range (this is likely an underestimate--should be able to be as high as 15.0 / node_spacing)
    max_stich_length = 10.0 / node_spacing

    # Might want to facilitate the initialization of a grid that fills
    # a larger area than is defined between the start and end points, just in case
    grid = Grid.oversized_grid(start_point, end_point, node_spacing=node_spacing, buffer_distance=buffer_distance)

    start_node = grid.nearest_node(start_point)
    end_node = grid.nearest_node(end_point)

    path = quickest_path(start_node, end_node, grid, include_diagonals=include_diagonals, euclidean=euclidean)
    stitched_path = stitch_colinear_nodes(start_node, path, maximum_stitch_size=max_stich_length)

    while len(stitched_path) != 0:

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
            path = quickest_path(rover_node, end_node, grid, include_diagonals=include_diagonals, euclidean=euclidean, verbose=verbose)
            stitched_path = stitch_colinear_nodes(rover_node, path, maximum_stitch_size=max_stich_length)
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
            recalculate_route = True
            continue

        if verbose:
            print("Move completed, currently predicting {} more waypoints".format(len(stitched_path)))
