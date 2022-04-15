import numpy as np
import helpers

'''
This is the framework for a coverage algorithm
This code does cover mapping.  It relies on a map already existing.
What we need:


Environment State:
    .map: a 2D array of nxm discrete points (x_i,y_i) that are either free (0) or occupied by obstacle (1).  Actual obstacles should be smaller, map should add a safety bumper to each obstacle.
    .res: resolution r of map, specifying physical distance between two adjacent points in meters
    .targets: target array containing q targets, their current estimated positions (x,y), and their state: Active = 1, Inactive = 0
    .robots: list of robots

Robot State:
    .id: robot id
    .pos: robot-specified position (x,y)
    .activity: robot current activity: Initializing = 0, Patrolling = 1, Hunting = 2


__Helper Functions__

move_robot(robot,new_location,Gazebo=False):
    moves a robot to a given location (x,y).
    avoids obstacles
    returns current position (x,y) of robot from odom topic, rounded to grid resolution
    returns False if new_location is unreachable
    If Gazebo, publish commands for Gazebo.  Else, publish for locobot.


'''

def do_coverage(environment,Gazebo=False,round_to=4):
    # Assume each robot has been placed in the testbed and is at a unique position.  Every robot's state is updated accordingly.
    # This function will update each robot's position to what it should be at a given time step and call move_robot() to move the robot.
    # This function will only try to move robots that are currently Patrolling

    # List of robots available for patrol
    patrollers = []
    for robot in environment.robots:
        if robot.activity == 1:
            patrollers.append(robot)
    
    # Initialize lists to store current robot positions and next positions
    robot_positions = [(0,0)] * len(patrollers)
    new_positions = [(0,0)] * len(patrollers)

    # Store current positions, Generate and store next positions
    for index,robot in enumerate(patrollers):
        robot_pos_x = round(robot.pos[0],round_to)
        robot_pos_y = round(robot.pos[1],round_to)
        robot_positions[index] = (robot_pos_x,robot_pos_y)
    # Generate and store next positions
    for index,robot in enumerate(patrollers):
        new_positions[index] = next_move(environment,robot,round_to)

    print("Should say: [(x0,y0),(x1,y1),(x2,y2)]")
    print("Actually says: ")
    print(robot_positions)

    # Don't return until every robot is at the required position in Hardware/Gazebo
    # Update each robot's state in the code
    for i in range(len(patrollers)):
        while robot_positions[i] != new_positions[i]: # Need to check if it is possible to get to exact location
            for index,robot in enumerate(patrollers):
                status = helpers.move_robot(robot,new_positions[index],Gazebo)
                if status == False:
                    print("Error: Obstacle. Quitting Coverage.")
                    return "Error: Obstacle. Quitting Coverage."
                else:
                    robot_positions[index] = status
                    robot.pos[0] = round(status[0],round_to)
                    robot.pos[1] = round(status[1],round_to)
    return

def next_move(environment,robot,round_to=4):
    robot_pos_x = round(robot.pos[0],round_to)
    robot_pos_y = round(robot.pos[1],round_to)
    best_move = np.random.choice([(1,0),(1,1),(1,-1),(-1,0),(-1,1),(-1,-1),(0,0),(0,1),(0,-1)])
    best_move[0] = round((best_move[0]*environment.res) + robot_pos_x,round_to)
    best_move[1] = round((best_move[1]*environment.res) + robot_pos_y,round_to)
    while not_legal(best_move):
        best_move = np.random.choice([(1,0),(1,1),(1,-1),(-1,0),(-1,1),(-1,-1),(0,0),(0,1),(0,-1)])
        best_move[0] = round((best_move[0]*environment.res) + robot_pos_x,round_to)
        best_move[1] = round((best_move[1]*environment.res) + robot_pos_y,round_to)
    new_x = best_move[0]
    new_y = best_move[1]

    def not_legal(move): # Return true if move is illegal
        robot_radius = .15 # meters
        move_x = move[0]
        move_y = move[1]
        x_index = round(move_x/environment.res)
        y_index = round(move_y/environment.res)
        if environment.map[x_index,y_index] == 1:
            # If there is an obstacle in the way
            return True
        for robot_ in environment.robots:
            if robot_ != robot:
                if np.sqrt((robot_.pos[0] - move[0])**2 + (robot_.pos[1] - move[1])**2) <= robot_radius:
                    # If a robot is nearby
                    return True
        return False

    return (new_x,new_y)