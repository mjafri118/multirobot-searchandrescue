from random import sample
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import numpy as np
from map_to_room_frame import map_to_room_frame, add_barrier_bumper_to_map
from scipy.spatial import distance  
import platform    # For getting the operating system name
import subprocess  # For executing a shell command


drop_point_offset_x = 0.0
drop_point_offset_y = 0.0

ROBOT_IN_MAP_NUMBER = 7
FREE_SPACE_IN_MAP_NUMBER = 0
UNKNOWN_SPACE_IN_MAP_NUMBER = -1
OCCUPIED_SPACE_IN_MAP_NUMBER = 100

def get_patrolling_locations(SEARCHER_CONFIGS, CURRENT_SEARCHER_IDX):
    # Decides best location for each searcher to go to
    # Note: this will be called continuously. Searchers will likely not reach destination before next call
    # This is a la self driving cars in path planning + constant adjustment (Position controller)

    # get locations
    searcher_locations = get_current_searcher_locations(SEARCHER_CONFIGS)
    
    # GET MAP DATA
    # 158 x  by 85 y
    map = add_barrier_bumper_to_map(map_to_room_frame(SEARCHER_CONFIGS[CURRENT_SEARCHER_IDX]['topic']))
    # generate random choices. 
    # for idx in range(158):
    #     for idy in range(85):
    #         map[idx,idy] = random.choices(population = [-1,0,25,100], weights= [.65, .25, .1])

    # switch out any algorithm here
    print("Running Neopolitan coverage algorithm")
    patrolling_goal_locations = neopolitan(searcher_locations, map)
    # patrolling_goal_locations = dontMove(searcher_locations, map)
    print('Ideal coverage location: ' + str(patrolling_goal_locations[CURRENT_SEARCHER_IDX]))
    return patrolling_goal_locations[CURRENT_SEARCHER_IDX]

# neopolitan algorithm
# divides map horizontally into "layers" where each LIVE robot must go to the center 
# of their designated layer. 
# returns patrolling goal locations for ALL robots (even the ones offline)
def neopolitan(searcher_locations, map):
    live_searcher_count = 0
    for searcher_location in searcher_locations:
        print(searcher_location)
        live_searcher_count += 1 if searcher_location != (-1,-1) else 0
    
    # catastrophic error 
    if live_searcher_count is 0:
        print("Trying to compute coverage algorithm with no searchers online!")
        return
    
    neopolitan_ideals = [(-1,-1)] * len(searcher_locations) 
    patrolling_goal_locations = []

    # we assume layers are split up by x direction. modularize in future.
    map_height = map.shape[1]

    layer_width = map.shape[0] / live_searcher_count
    layers_total = live_searcher_count
    layers_used = 0

    for searcher_idx in range(len(searcher_locations)):
        # do not allocate any of the layers to a robot that is offline
        if searcher_locations[searcher_idx] == (-1,-1):
            continue

        # neopolitan_ideals[searcher_idx] = room_frame_to_robot_frame((layers_used * layer_width + layer_width / 2, map_height/2))
        neopolitan_ideals[searcher_idx] = ((layers_used * layer_width) + (layer_width / 2), map_height/2)
        layers_used += 1
    #return neopolitan_ideals
    free_spaces = np.argwhere(map == FREE_SPACE_IN_MAP_NUMBER)
    
    # only go to target if feasible
    for neopolitan_ideal in neopolitan_ideals:
        # if searcher isn't going to take commands, don't give them any
        if neopolitan_ideal == (-1,-1):
            patrolling_goal_locations.append((-1,-1))
            continue

        print("NEOPOLITAN IDEAL:")
        print(room_frame_to_robot_frame(neopolitan_ideal))
            
        # if searcher is targetting a location that isn't reachable, then find nearest neighbor that isn't problematic
        if map[neopolitan_ideal[0], neopolitan_ideal[1]] in [UNKNOWN_SPACE_IN_MAP_NUMBER, OCCUPIED_SPACE_IN_MAP_NUMBER, ROBOT_IN_MAP_NUMBER]:
            # closest node algorithm as per https://codereview.stackexchange.com/questions/28207/finding-the-closest-point-to-a-list-of-points
            node = neopolitan_ideal
            nodes = np.asarray(free_spaces)
            # dist_2 = np.sum((nodes - node)**2, axis=1)
            # closest_index = distance.cdist([node], nodes).argmin()
            all_distances = distance.cdist([node], nodes)
            # Need to get the index of the minimum entry
            min_index = np.where(all_distances ==np.amin(all_distances))
            print(all_distances)
            print(min_index)
            min_index = int(min_index[1][0])
            closest_clean_node = nodes[min_index]
            patrolling_goal_locations.append((closest_clean_node[0], closest_clean_node[1]))
            
            print('NEOPOLITAN IDEAL IS NOT ACHIEVABLE. CLOSEST: ')
            print(room_frame_to_robot_frame((closest_clean_node[0], closest_clean_node[1])))

        # else, searcher can go to neopolitan ideal because the space is free
        else:
            print("NAVIGATING TO NEOPOLITAN IDEAL")
            patrolling_goal_locations.append(neopolitan_ideal)

    return [room_frame_to_robot_frame(patrolling_goal_location) for patrolling_goal_location in patrolling_goal_locations]

# optimal coverage location is truly just where it currently is. 
def dontMove(searcher_locations, map):
    patrolling_goal_locations = [(-1,-1)] * len(searcher_locations) 

    for i in range(len(searcher_locations)):
        # do not allocate any of the layers to a robot that is offline
        if searcher_locations[i] == (-1,-1):
            continue
        patrolling_goal_locations[i] = room_frame_to_robot_frame(searcher_locations[i]) 

    return patrolling_goal_locations  

def is_ip_online(IP):
    # Ping the TX node's ip address, return true/false depending on if ping went through. 
    # use 
    # Option for the number of packets as a function of
    param = '-n' if platform.system().lower()=='windows' else '-c'

    # Building the command. Ex: "ping -c 1 google.com"
    command = ['ping', param, '1', IP]

    ip_sensed = subprocess.call(command, stdout=subprocess.PIPE) == 0

    return ip_sensed

# gets where all available robots are in ROOM FRAME. 
# note, if the robot is not online, their location is (-1, -1)
def get_current_searcher_locations(SEARCHER_CONFIGS):
    print("Getting robot current searcher locations")
    searcher_locations = [(None,None)]* len(SEARCHER_CONFIGS)

    for i in range(len(SEARCHER_CONFIGS)):
        try:
            # save wasted timeout time if we cannot ping
            if is_ip_online(SEARCHER_CONFIGS[i]['target_ip']):
                topic_response = rospy.wait_for_message(SEARCHER_CONFIGS[i]['topic'] + '/mobile_base/odom', Odometry, timeout=30)
                searcher_locations[i] = robot_frame_to_room_frame((topic_response.pose.pose.position.x, topic_response.pose.pose.position.y))
            else: 
                print("Agent is not online: " + SEARCHER_CONFIGS[i]['topic'])
                searcher_locations[i] = (-1,-1)

        # If an agent is not publishing their odometry, just continue
        # This controls for case if a robot dies. Algorithm should proceed.
        except rospy.exceptions.ROSException:
            print("Can't find location for "+ SEARCHER_CONFIGS[i]['topic'])
            searcher_locations[i] = (-1,-1)
    print("Returning robot current searcher locations")
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