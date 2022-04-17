# before we run this script: 
# each Searcher starts at the same physical origin and is manually sent into a region of the map
# run this script once all Searchers have reached an initial position, all relative to the same starting point.

from operator import indexOf
from searcher import Searcher

TARGET_IP_ADDRESS = '192.168.1.30'
STATUS_OPTIONS = ['hunting', 'patrolling', 'listening', 'idle']

def loop():
    pass

def main():
    searcher1 = Searcher('searcher1', TARGET_IP_ADDRESS)
    searcher2 = Searcher('searcher2', TARGET_IP_ADDRESS)
    searcher3 = Searcher('searcher3', TARGET_IP_ADDRESS)
    searchers = [searcher1, searcher2, searcher3]

    loop()

    # think slower on this. 
    while (searcher.is_target_sensed() is False for searcher in searchers):
        # do patrolling
        perform_patrolling(searchers)
    
    # if we reached here, then someone has detected a node. 
    # this means that ALL searchers go to listening state
    for searcher in searchers: 
        searcher.status == STATUS_OPTIONS[2]
        searcher.update_aoa_reading()
    
    # select the hunter
    hunter_index = indexOf(max([searcher.aoa_strength] for searcher in searchers))
    hunter = searchers[hunter_index]

    # hunter performs hunting 
    hunter.move_robot_in_direction(hunter.aoa_angle)
    while hunter.is_moving():



def perform_patrolling(searchers):
    # Task 1: decide best location for each searcher to go to
    # Note: this will be called continuously. Searchers will likely not reach destination before next call
    # This is a la self driving cars in path planning + constant adjustment (P controller)
    # TODO: algorithm for task 1
    locations = [(0,1,1), (2,3,4), (1,4,6)] # of length searchers

    # Task 2: move robots to those locations via searcher.move_robot_to_waypoint()
    for i, searcher in enumerate(searchers): 
        searcher.move_robot_to_waypoint(locations[i])


if __name__ == "__main__":
    main()