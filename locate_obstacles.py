from math import isinf, pi, sin, cos, radians

def locate_obstacles(rover, sweep_angle=pi/2):
    """
    Identifies obstacles and calculates their location in (x, y) form

    Parameters
    - rover: the Gazebo rover
    - sweep_angle: the angle through which the LiDAR beams are emitted

    Returns a list of (x, y) tuples
    """
    num_lidar_sensors = len(rover.laser_distances)
    obstacles = []

    for i in range(0, num_lidar_sensors):

        distance = rover.laser_distances[i]

        if not isinf(distance):
            # The angle of the lidar sensor with respect to the rover
            lidar_angle = sweep_angle * (0.5 - i/(num_lidar_sensors - 1))
            obstalce_heading = radians(rover.heading) + lidar_angle

            obstacle_x = rover.x + distance * cos(obstalce_heading)
            obstacle_y = rover.y + distance * sin(obstalce_heading)

            obstacles.append((obstacle_x, obstacle_y))

    return obstacles