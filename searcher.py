import random


STATUS_OPTIONS = ['hunting', 'patrolling', 'listening', 'idle']

class Searcher: 
    def __init__(self, name, target_ip_address):
        # any human readable name
        self.name = name
        self.target_ip_address = target_ip_address
        # publish:

        self.location = (random.uniform(0,1), random.uniform(0,1), random.uniform(0,1))
        self.aoa_angle = 0 # bounded to [-180, 180]
        self.aoa_strength = 0 # bounded to [0, 1]

        self.status = STATUS_OPTIONS[1]

        self.map = None # don't access this variable. always call get_map() for most up-to-date map.

    def get_map(self):
        # TODO
        return None
        
    def is_target_sensed(self):
        # TODO: ping the TX node's ip address, return true/false depending on if ping went through. 
        # use self.target_ip_address
        if self.status != 'patrolling':
            print("Can't update target sensing when not in listening state.")
            return
        self.target_sensed = random.choice([0,1])
    
    def update_aoa_reading(self):
        # TODO: actually update AOA reading
        # this will require the robot to be fully stopped, and rotate itself. 
        self.aoa_angle = random.uniform(-180,180)
        self.aoa_strength = random.uniform(0,1)
    
    def stop_robot(self):
        # TODO: stops robot from movement
        self.status = 'idle'
        return
    
    def obstacle_detected(self):
        # TODO
        return random.choice([0,1])

    def move_robot_in_direction(self, aoa_angle):
        # used in hunting state
        # note: make sure obstacle avoidance is there
        # /locobot/mobile_base/commands/velocity geometry_msgs/T
        while not self.obstacle_detected():
            # keep moving in direction of aoa_angle
            continue
        
        # if we reach here, we have detected on obstacle
        self.transition_to_listening()
    
    def update_status(self, next_status):
        self.status =  next_status
    
    def move_robot_to_waypoint(self, waypoint):
        # TODO: give a discrete (x,y) location for robot to travel to with obstacle avoidance
        #  this should just send a publish, i.e. do not make this function wait until reaching the waypoint before completion
        pass

    def is_moving(self):
        # TODO: this can be inferred from reading the pose velocity
        # specifically, topic:  
        # return true or false
        return False