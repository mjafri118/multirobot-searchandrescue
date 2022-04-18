from searcher import Searcher
from patrol import get_patrolling_locations
import rospy
from std_msgs.msg import Bool, Float32

# code is currently built to support THREE searcher agents exactly. 
SEARCHER_CONFIGS = [
    {
        "name": 'searcher1',
        "topic": 'locobot'
    },
    {
        "name": 'searcher2',
        "topic": 'locobot2'
    },
    {
        "name": 'searcher3',
        "topic": 'locobot3'
    }
]

CURRENT_SEARCHER_IDX = 0

TARGET_NODE_IP = '192.168.1.30'

class SearcherFSM:
    def __init__(self):
        self.S = Searcher(SEARCHER_CONFIGS[CURRENT_SEARCHER_IDX]['name'], SEARCHER_CONFIGS[CURRENT_SEARCHER_IDX]['topic'], TARGET_NODE_IP)

        # setup callback subscribers to other searchers' is_target_sensed
        other_searcher_1_config = SEARCHER_CONFIGS[(CURRENT_SEARCHER_IDX + 1) % 3]
        other_searcher_2_config =  SEARCHER_CONFIGS[(CURRENT_SEARCHER_IDX + 2) % 3]
        rospy.Subscriber(other_searcher_1_config['topic'] + "/target_node_sensed", Bool, self.cb1_target_node_sensed)
        rospy.Subscriber(other_searcher_2_config['topic'] + "/target_node_sensed", Bool, self.cb2_target_node_sensed)

        rospy.Subscriber(other_searcher_1_config['topic'] + "/aoa_strength", Float32, self.cb1_aoa_strength_update)
        rospy.Subscriber(other_searcher_2_config['topic'] + "/aoa_strength", Float32, self.cb2_aoa_strength_update)
        
        self.current_state = 'patrolling'   
        print("Initialization successful!")    
        print("First state: " + self.current_state) 

        # In case the callbacks below are never invoked, i.e. other searchers are non-functional somehow.
        self.other_searcher1_target_node_sensed = False
        self.other_searcher1_aoa_strength = 0
        self.other_searcher2_target_node_sensed = False
        self.other_searcher2_aoa_strength = 0
    
    def cb1_target_node_sensed(self, node_sensed):
        self.other_searcher1_target_node_sensed = node_sensed
    
    def cb2_target_node_sensed(self, node_sensed):
        self.other_searcher2_target_node_sensed = node_sensed

    def cb1_aoa_strength_update(self, data):
        self.other_searcher1_aoa_strength = data
    
    def cb2_aoa_strength_update(self, data):
        self.other_searcher2_aoa_strength = data

    def isDemoted(self):
        if (self.S.aoa_strength < self.other_searcher1_aoa_strength) or (self.S.aoa_strength < self.other_searcher2_aoa_strength):
            return True
        else:
            return False
    

    def loop(self):
        while True:
            next_state = self.current_state # by default, stays within the current state

            if self.current_state is 'patrolling':
                # Perform normal state task             
                # Get the locations that robots should go to
                # Task 2: move robot to location via move_robot_to_waypoint()
                if not self.S.is_moving():
                    patrolling_location_goal = get_patrolling_locations(SEARCHER_CONFIGS, CURRENT_SEARCHER_IDX)
                    self.S.move_robot_to_waypoint(patrolling_location_goal)

                # Exit conditions
                if self.S.is_target_sensed() or self.other_searcher1_target_node_sensed or self.other_searcher2_target_node_sensed:
                    next_state = 'listening'

            if self.current_state is 'listening':
                # Perform normal state task, be sure to publish aoa data to the other robots
                # AOA angles should be in global frame, not robot frames
                self.S.update_aoa_reading()

                # Exit conditions
                if self.S.aoa_strength > self.other_searcher1_aoa_strength and self.S.aoa_strength > self.other_searcher2_aoa_strength:
                    next_state = 'hunting'
            
            if self.current_state is 'hunting':
                # Perform normal state task
                error_threshold = 5 # degrees
                # first, orient the robot in the proper angle so it is headed at the target
                deg_to_target = self.S.get_location()[2] - self.S.aoa_angle

                # if robot is not oriented within threshold
                if abs(deg_to_target) > error_threshold:
                    self.S.move_robot_in_direction(linear=False, positive=deg_to_target < 0)
                
                # second, linearly move the robot towards the target
                else:
                    self.S.move_robot_in_direction(linear=True,positive=True)

                # Exit conditions
                if self.S.obstacle_detected() or self.isDemoted():
                    next_state = 'listening'
            
            # if self.current_state is 'idle':
            #     # Perform normal state task
            #     if manualResetHit:
            #         next_state = 'patrolling'
            if(self.current_state != next_state):
                print("Changing state to: " + next_state)

            self.current_state = next_state

if __name__ == "__main__":
    F = SearcherFSM()
    while True: 
        F.loop()