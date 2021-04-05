from qset_lib import Rover
from run_course import run_course

rover = Rover()
end_point = (10, 10)
node_spacing = 0.4
obstacle_padding = 0.4

run_course(rover, end_point, node_spacing=node_spacing, obstacle_padding=obstacle_padding, verbose=True)