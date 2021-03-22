from pathfinding import quickest_path
from move_rover import move_rover
from locate_obstacles import locate_obstacles
from grid_node import distance, neighbouring_nodes
from grid import Grid

def run_course(rover, end_point, side_length=1, include_diagonals=True, euclidean=True, verbose=False):
    
    recalculate_route = False
    start_point = (rover.x, rover.y)

    # Might want to facilitate the initialization of a grid that fills
    # a larger area than is defined between the start and end points, just in case
    grid = Grid(start_point, end_point, side_length=side_length)

    start_node = grid.nearest_node(start_point)
    end_node = grid.nearest_node(end_point)

    path = quickest_path(start_node, end_node, grid, include_diagonals=include_diagonals, euclidean=euclidean)

    while len(path) != 0:

        obstacles = locate_obstacles(rover)

        for obstacle in obstacles:
            if not grid.nearest_node(obstacle).is_obstacle:
                grid.nearest_node(obstacle).is_obstacle = True
                for node in neighbouring_nodes(obstacle, grid, include_diagonals=include_diagonals):
                    # Includes all neighbouring nodes as obstacles
                    # Does not count the rover as an obstacle if it is nearbys
                    if node is not grid.nearest_node((rover.x, rover.y)):
                        node.is_obstacle = True
                recalculate_route = True
        
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
            print("Moving to location {} from {}".format((round(next_location[0], 2), round(next_location[1], 2)), (round(rover.x, 2), round(rover.y))))
        move_rover(rover, next_location[0], next_location[1])
        if verbose:
            print("Move completed, currently predicting {} more waypoints".format(len(path)))
