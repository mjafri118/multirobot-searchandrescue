from random import sample
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import numpy as np
from map_to_room_frame import map_to_room_frame

drop_point_offset_x = 0.0
drop_point_offset_y = 0.0

def get_patrolling_locations(SEARCHER_CONFIGS, CURRENT_SEARCHER_IDX):
    # Decides best location for each searcher to go to
    # Note: this will be called continuously. Searchers will likely not reach destination before next call
    # This is a la self driving cars in path planning + constant adjustment (P controller)

    # get locations
    searcher_locations = get_current_searcher_locations(SEARCHER_CONFIGS)
    
    # GET MAP DATA
    # 158 x  by 85 y
    map = map_to_room_frame(SEARCHER_CONFIGS[CURRENT_SEARCHER_IDX]['topic'])
    print(map)
    # generate random choices. 
    # for idx in range(158):
    #     for idy in range(85):
    #         map[idx,idy] = random.choices(population = [-1,0,25,100], weights= [.65, .25, .1])

    # switch out any algorithm here
    patrolling_goal_locations = neopolitan(searcher_locations, map)
    # patrolling_goal_locations = dontMove(searcher_locations, map)
    print("PATROLLING GOAL LOCATIONS BELOW: ")
    print(patrolling_goal_locations)
    
    return patrolling_goal_locations[CURRENT_SEARCHER_IDX]

# neopolitan algorithm
# divides map horizontally into "layers" where each LIVE robot must go to the center 
# of their designated layer. 
# returns patrolling goal locations for ALL robots (even the ones offline)
def neopolitan(searcher_locations, map):
    live_searcher_count = 0
    for searcher_location in searcher_locations:
        live_searcher_count += 1 if searcher_location != (-1,-1) else 0
    print('live_searcher_count is (expecting 1) ' + str(live_searcher_count)) 
    
    # catastrophic error 
    if live_searcher_count is 0:
        print("Trying to compute coverage algorithm with no searchers online!")
        return
    
    patrolling_goal_locations = [(-1,-1)] * len(searcher_locations) 

    # we assume layers are split up by x direction. modularize in future.
    map_height = map.shape[1]
    print('map_height is (expecting 85) ' + str(map_height)) 

    layer_width = map.shape[0] / live_searcher_count
    print('layer_width is (expecting 158) ' + str(layer_width)) 
    layers_total = live_searcher_count
    layers_used = 0

    for searcher_idx in range(len(searcher_locations)):
        # do not allocate any of the layers to a robot that is offline
        if searcher_locations[searcher_idx] == (-1,-1):
            continue

        patrolling_goal_locations[searcher_idx] = room_frame_to_robot_frame((layers_used * layer_width + layer_width / 2, map_height/2))
        layers_used += 1

    return patrolling_goal_locations

# optimal coverage location is truly just where it currently is. 
def dontMove(searcher_locations, map):
    patrolling_goal_locations = [(-1,-1)] * len(searcher_locations) 

    for i in range(len(searcher_locations)):
        # do not allocate any of the layers to a robot that is offline
        if searcher_locations[i] == (-1,-1):
            continue
        patrolling_goal_locations[i] = room_frame_to_robot_frame(searcher_locations[i]) 

    return patrolling_goal_locations  


# gets where all available robots are in ROOM FRAME. 
# note, if the robot is not online, their location is (-1, -1)
def get_current_searcher_locations(SEARCHER_CONFIGS):
    searcher_locations = [(None,None)]* len(SEARCHER_CONFIGS)

    for i in range(len(SEARCHER_CONFIGS)):
        try:
            topic_response = rospy.wait_for_message(SEARCHER_CONFIGS[i]['topic'] + '/mobile_base/odom', Odometry, timeout=1) 
            searcher_locations[i] = robot_frame_to_room_frame((topic_response.pose.pose.position.x, topic_response.pose.pose.position.y))
            print(searcher_locations[i])
        
        # If an agent is not publishing their odometry, just continue
        # This controls for case if a robot dies. Algorithm should proceed.
        except rospy.exceptions.ROSException:
            searcher_locations[i] = (-1,-1)
    
    return searcher_locations

# if robot is at where it begins,
# return 22.5 ft -> 6.85 m -> 137, 3.25 ft -> 1 m -> 20 so [137, 20]
def robot_frame_to_room_frame(robot_frame):
    return (
        int(round((robot_frame[0] + 6.85 - drop_point_offset_x)/.05)), 
        int(round((robot_frame[1] + 1 + drop_point_offset_y)/.05)),
    )

def room_frame_to_robot_frame(room_frame):
    return (
        round(room_frame[0] * .05 + drop_point_offset_x - 6.85, 2),
        round(room_frame[1] * .05 - drop_point_offset_y - 1, 2),
    )    