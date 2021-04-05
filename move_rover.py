from math import atan2, radians, sqrt
from errors import ObseleteGridError
from locate_obstacles import locate_obstacles

def move_rover(rover, x, y, grid):
    """
    Moves a rover from its current location to a specified destination

    Parameters
    ----------
    rover : Rover
        The rover within the Gazebo simulation
    x: float
        The x coordinate of the desired destination
    - y: float
        The y coordinate of the desired destination

    Raises
    ------
    ObseleteGridError
        if untracked obstacles are found on the rover's path 
    """
    angle_to_travel = atan2(y - rover.y, x - rover.x)
    angle_tolerance = 5e-2
    # Turn the rover to face the desired direction
    while abs(radians(rover.heading) - angle_to_travel) > angle_tolerance:
        # Speed is proportional to how much angular distance there is still to travel, with a minimum speed
        rover.send_command(0, angle_to_travel - radians(rover.heading))
        
        for obstacle_pos in locate_obstacles(rover):
            obstacle_node = grid.nearest_node(obstacle_pos)
            
            if not obstacle_node.is_obstacle:
                raise ObseleteGridError

    rover.send_command(0, 0)

    # Moves the rover forward
    distance_to_travel = sqrt((x - rover.x) ** 2 + (y - rover.y) ** 2)
    speed_factor = 2
    dist_tolerance = 1e-1
    wrong_direction_counter = 0

    while abs(distance_to_travel) > dist_tolerance:
        distance_remaining = sqrt((x - rover.x) ** 2 + (y - rover.y) ** 2)
        # Occurs if rover is getting further from its destination
        if distance_remaining > distance_to_travel:
            wrong_direction_counter += 1
        elif distance_remaining < distance_to_travel and wrong_direction_counter > 0:
            wrong_direction_counter -= 1

        if wrong_direction_counter == 5:
            break

        distance_to_travel = distance_remaining
        
        # Speed is capped at 0.25, as velocity proportional to position results in large overshoots over longer distances
        # Theoretically, the diminishing speed as the rover nears its target is useful for smooth deceleration, but in practice it's unclear
        speed = min(speed_factor * distance_to_travel, 0.25)
        rover.send_command(speed, 0)   

    rover.send_command(0, 0)
