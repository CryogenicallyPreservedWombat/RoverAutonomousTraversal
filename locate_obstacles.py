from math import isinf, pi, sin, cos, radians

def locate_obstacles(rover, sweep_angle=pi/2, sensors_to_ignore=[]):
    """
    Identifies obstacles and calculates their location in (x, y) form

    Parameters
    - rover: the Gazebo rover
    - sweep_angle: the angle through which the LiDAR beams are emitted

    Returns a list of (x, y) tuples
    """
    num_lidar_sensors = len(rover.laser_distances)
    obstacles = []

    for i in range(num_lidar_sensors):

        if i in sensors_to_ignore:
            continue

        distance = rover.laser_distances[i]

        if not isinf(distance):
            # The angle of the lidar sensor with respect to the rover
            # Casting i to a float is necessary in Python 2.x
            lidar_angle = sweep_angle * (-0.5 + float(i) / (num_lidar_sensors - 1))
            obstacle_heading = radians(rover.heading) + lidar_angle

            obstacle_x = rover.x + distance * cos(obstacle_heading)
            obstacle_y = rover.y + distance * sin(obstacle_heading)

            obstacles.append((obstacle_x, obstacle_y))

    return obstacles