from nav_msgs.msg import OccupancyGrid
import numpy as np
import rospy
import matplotlib.pyplot as plt

ROBOT_RADIUS = 5 # 8 cells across is a reasonable buffer

def map_to_room_frame(topic):
    # print("Mapping to room frame...")
    drop_point_offset_x = 0.0 # if closer to left wall of room, offset should be positive
    drop_point_offset_y = 0.0 # if closer to back wall of room, offset should be positive

    map = rospy.wait_for_message('/'+ topic+'/rtabmap/grid_map', OccupancyGrid)
    # print("Received map")
    #res = map.info.resolution # m/cell
    width = map.info.width # cells
    height = map.info.height # cells
    actual_map_origin_x = map.info.origin.position.x
    actual_map_origin_y = map.info.origin.position.y
    #origin_angle = np.rad2deg(euler_from_quaternion(map.info.origin.orientation.z))
    data = map.data # the map data, in row-major order, starting with (0,0).  Occupancy probabilities are in range [0,100]. Unknown is -1
    res = 0.05 # Meters

    testbed_w = 7.92 # meters wide (heading of robot at initialization)
    testbed_h = 4.27 # meters tall
    testbed_w_cells = int(round(testbed_w/res))
    testbed_h_cells = int(round(testbed_h/res))
    smaller_map = np.zeros((testbed_w_cells,testbed_h_cells))

    origin_x = actual_map_origin_x # Should be negative
    origin_y = actual_map_origin_y # Should be negative
    origin_x_grid = int(round(origin_x/res))
    origin_y_grid = int(round(origin_y/res))

    lower_left_x = (-6.86 + drop_point_offset_x) - origin_x
    lower_left_y = (-0.99 - drop_point_offset_y) - origin_y
    lower_left_x_grid = int(round(lower_left_x/res))
    lower_left_y_grid = int(round(lower_left_y/res))

    upper_right_x_grid = lower_left_x_grid + testbed_w_cells - 1
    upper_right_y_grid = lower_left_y_grid + testbed_h_cells - 1

    iter = 0
    # Saving smaller map by limiting scope
    for row in range(height-1,-1,-1):
        for column in range(width):
            index = ((width)*(row)) + column
            if row == -origin_y_grid and column == -origin_x_grid:
                # If the robot is here
                small_column = iter % testbed_w_cells
                small_row = int((iter - small_column)/testbed_w_cells)
                smaller_map[small_column,testbed_h_cells - 1 - small_row] = 7 # Value for robot taking up a space
                iter+=1

            elif row >= lower_left_y_grid and row <= upper_right_y_grid and column >= lower_left_x_grid and column <= upper_right_x_grid:
                small_column = iter % testbed_w_cells
                small_row = int((iter - small_column)/testbed_w_cells)
                smaller_map[small_column,testbed_h_cells - 1 - small_row] = data[index]
                iter+=1
    
    #plt.imshow(smaller_map,interpolation='nearest')
    #plt.show()
    return smaller_map

def add_barrier_bumper_to_map(map):
    width = map.shape[0]
    height = map.shape[1]

    for idx in range(width):
        for idy in range(height):
            if map[idx,idy] == 100:
                for neighbor_x in range(idx-ROBOT_RADIUS,idx+ROBOT_RADIUS):
                    for neighbor_y in range(idy-ROBOT_RADIUS,idy+ROBOT_RADIUS):
                        if neighbor_x < width and neighbor_x >= 0:
                            if neighbor_y < height and neighbor_y >= 0:
                                if map[neighbor_x,neighbor_y] != 100:
                                    map[neighbor_x,neighbor_y] = 50
    for idx in range(width):
        for idy in range(height):
            if map[idx,idy] == 50:
                map[idx,idy] = 100
    #plt.imshow(map,interpolation='nearest')
    #plt.show()
    return map