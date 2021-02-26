from math import atan2, radians

def move_rover(rover, x, y):
    """
    Moves a rover from its current location to a point (x, y)
    """
    angle_to_travel = atan2(y - rover.y, x - rover.x)
    angle_tolerance = 1e-2
    # Turn the rover to face the desired direction
    while abs(radians(rover.heading) - angle_to_travel) > angle_tolerance:
        # Speed is proportional to how much angular distance there is still to travel
        rover.send_command(0, angle_to_travel - radians(rover.heading))
    rover.send_command(0, 0)
    
    dist_tolerance = 1e-3
    # Move the rover to its destination
    while abs(rover.x - x) < dist_tolerance and abs(rover.y - y) < dist_tolerance:
        rover.send_command(1, 0)
    
    rover.send_command(0, 0)

help(move_rover)