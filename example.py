from qset_lib import Rover
from run_course import run_course

rover = Rover()
end_point = (10, 10)
side_length = 0.3
obstacle_padding = 0.3

run_course(rover, end_point, side_length=side_length, obstacle_padding=obstacle_padding, verbose=True)