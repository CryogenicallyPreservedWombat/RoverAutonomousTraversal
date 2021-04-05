from math import atan2, radians, sqrt

def move_rover(rover, x, y):
    """
    Moves a rover from its current location to a specified destination

    Parameters
    - rover: the Gazebo rover
    - x: the desired x coordinate
    - y: the desired y coordinate
    """
    angle_to_travel = atan2(y - rover.y, x - rover.x)
    angle_tolerance = 1e-1
    # Turn the rover to face the desired direction
    while abs(radians(rover.heading) - angle_to_travel) > angle_tolerance:
        # Speed is proportional to how much angular distance there is still to travel
        rover.send_command(0, angle_to_travel - radians(rover.heading))
    rover.send_command(0, 0)
    
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
        
        # Having speed be proportional to distance remaining seems to work for smaller distances
        # For larger distances it tends to overshoot drastically, which is why speed is capped at 0.5
        speed = min(speed_factor * distance_to_travel, 0.5)
        print(speed)
        rover.send_command(speed, 0)   

    rover.send_command(0, 0)
