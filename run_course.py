from pathfinding import quickest_path
from move_rover import move_rover
from locate_obstacles import locate_obstacles
from grid_node import distance, neighbouring_nodes
from grid import Grid
from math import ceil

def run_course(rover, end_point, side_length=1, include_diagonals=True, euclidean=True, verbose=False, sensors_to_ignore=[7], obstacle_padding=0.5):
    
    recalculate_route = False
    start_point = (rover.x, rover.y)

    # Might want to facilitate the initialization of a grid that fills
    # a larger area than is defined between the start and end points, just in case
    grid = Grid(start_point, end_point, side_length=side_length)

    start_node = grid.nearest_node(start_point)
    end_node = grid.nearest_node(end_point)

    path = quickest_path(start_node, end_node, grid, include_diagonals=include_diagonals, euclidean=euclidean)

    while len(path) != 0:

        obstacles = locate_obstacles(rover, sensors_to_ignore=sensors_to_ignore)

        for obstacle in obstacles:
            obstacle_node = grid.nearest_node(obstacle)

            if not obstacle_node.is_obstacle:

                obstacle_node.is_obstacle = True
                recalculate_route = True

                # Calculates the appropriate amount of padding to give each obstacle
                padding_layers = int(ceil(float(obstacle_padding) / grid.side_length))

                for node in neighbouring_nodes(obstacle_node, grid, include_diagonals=include_diagonals, radius=padding_layers):
                    if node is not grid.nearest_node((rover.x, rover.y)):
                        node.is_padding = True


        if recalculate_route:
            if verbose:
                print("Found new obstacles, recalculating route")
            # might want to create an entirely new grid based on the current rover's position
            current_node = grid.nearest_node((rover.x, rover.y))
            current_node.is_rover = True
            path = quickest_path(current_node, end_node, grid, include_diagonals=include_diagonals, euclidean=euclidean, verbose=verbose)
            recalculate_route = False
        
        if verbose:
            print(grid)
            current_node.is_rover = False

        next_node = path.pop(0)
        row, column = next_node.coords
        next_location = grid.location(row, column)
        if verbose:
            print("Moving to location {} from {}".format((round(next_location[0], 2), round(next_location[1], 2)), (round(rover.x, 2), round(rover.y, 2))))
        move_rover(rover, next_location[0], next_location[1])
        if verbose:
            print("Move completed, currently predicting {} more waypoints".format(len(path)))
