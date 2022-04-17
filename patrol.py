from random import sample
import rospy

def get_patrolling_locations(SEARCHER_CONFIGS, CURRENT_SEARCHER_IDX):
    # Decides best location for each searcher to go to
    # Note: this will be called continuously. Searchers will likely not reach destination before next call
    # This is a la self driving cars in path planning + constant adjustment (P controller)

    # get locations
    searcher_locations = [(0,0,0)]* len(SEARCHER_CONFIGS)
    for i in range(SEARCHER_CONFIGS): 
        searcher_locations[i] = rospy.oneTimeSubscriber(SEARCHER_CONFIGS[i], location)
    
    # TODO: algorithm for task 1
    sample_patrolling_goal_location = searcher_locations # of length searchers
    return sample_locations[CURRENT_SEARCHER_IDX]

    