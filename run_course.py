from pathfinding import quickest_path
from move_rover import move_rover
from locate_obstacles import locate_obstacles
from grid_node import distance
from grid import Grid

def run_course(rover, end_point, include_diagonals=True, euclidean=True, verbose=False):
    
    # parameters should be fine-tuned
    side_length = 1

    # parameters shouldn't have to be fine-tuned
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
                recalculate_route = True
        
        if recalculate_route:
            if verbose:
                print("Found new obstacles, recalculating route")
            # might want to create an entirely new grid based on the current rover's position
            current_node = grid.nearest_node((rover.x, rover.y))
            path = quickest_path(current_node, end_node, grid, include_diagonals=include_diagonals, euclidean=euclidean)

        next_node = path.pop(0)
        row, column = next_node.coords
        next_location = grid.location(row, column)
        if verbose:
            print("Moving to location {loc}".format(next_location))      
        move_rover(rover, next_location[0], next_location[1])
        if verbose:
            print("Move completed, currently predicting {num} more waypoints".format(len(path)))

