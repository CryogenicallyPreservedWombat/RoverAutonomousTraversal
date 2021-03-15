from qset_lib import Rover
from run_course import run_course

rover = Rover()
# default value, client has not yet specified in what form the end coordinate will be provided
run_course(rover, end_point=(10, 10), verbose=True)