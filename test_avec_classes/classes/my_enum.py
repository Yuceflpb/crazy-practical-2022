from enum import Enum

class State(Enum):
    take_off_from_base = 0
    go_to_target_zone = 1
    search_target = 2
    refine_target = 3
    landing_target = 5
    wait_after_landing_target = 6
    take_off_from_target = 7
    go_to_base_loc = 8
    search_base = 9
    refine_base = 10
    landing_base = 11
    exit = 12

    error = -1

    debug_refine_target = -3

class Direction(Enum):
    forward = 0
    back = 1
    right = 2
    left = 3