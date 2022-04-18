from random import sample
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import numpy as np

def get_patrolling_locations(SEARCHER_CONFIGS, CURRENT_SEARCHER_IDX):
    # Decides best location for each searcher to go to
    # Note: this will be called continuously. Searchers will likely not reach destination before next call
    # This is a la self driving cars in path planning + constant adjustment (P controller)

    # get locations
    searcher_locations = [(None,None,None)]* len(SEARCHER_CONFIGS)

    for i in range(len(SEARCHER_CONFIGS)):
        try:
            topic_response = rospy.wait_for_message(SEARCHER_CONFIGS[i]['topic'] + '/mobile_base/odom', Odometry, timeout=1) 
            orientation = (topic_response.pose.pose.orientation.x, 
                topic_response.pose.pose.orientation.y, 
                topic_response.pose.pose.orientation.z, 
                topic_response.pose.pose.orientation.w)
            searcher_locations[i] = (topic_response.pose.pose.position.x, topic_response.pose.pose.position.y, np.rad2deg(euler_from_quaternion(orientation)[2]))
        
        # If an agent is not publishing their odometry, just continue
        # This controls for case if a robot dies. Algorithm should proceed.
        except rospy.exceptions.ROSException:
            continue

    # TODO: algorithm for task 1
    sample_patrolling_goal_location = searcher_locations # of length searchers
    return sample_patrolling_goal_location[CURRENT_SEARCHER_IDX]

    