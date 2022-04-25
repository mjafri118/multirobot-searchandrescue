# Terminal command to be run on locobot3: python searcherFSM.py --robot_index 0
# Terminal command to be run on locobot4: python searcherFSM.py --robot_index 1
# Terminal command to be run on locobot5: python searcherFSM.py --robot_index 2

from searcher import Searcher
from patrol import get_patrolling_locations
import rospy
from std_msgs.msg import Bool, Float32
import argparse

# code is currently built to support THREE searcher agents exactly. 
SEARCHER_CONFIGS = [
    {
        "name": 'searcher1',
        "topic": 'locobot3',
        "target_ip": '192.168.1.29'
    },
    {
        "name": 'searcher2',
        "topic": 'locobot4',
        "target_ip": '192.168.1.23'
    },
    {
        "name": 'searcher3',
        "topic": 'locobot5',
        "target_ip": '192.168.1.30'
    }
]

MAX_SIGNAL_STRENGTH_VIA_VARIANCE = 10000

WAYPOINT_THRESHOLD = [.15,.15]

#TARGET_NODE_IP = '192.168.1.30'

class SearcherFSM:
    def __init__(self, robot_index):
        self.CURRENT_SEARCHER_IDX = robot_index
        self.S = Searcher(SEARCHER_CONFIGS[self.CURRENT_SEARCHER_IDX]['name'], SEARCHER_CONFIGS[self.CURRENT_SEARCHER_IDX]['topic'], SEARCHER_CONFIGS[self.CURRENT_SEARCHER_IDX]['target_ip'])

        # setup callback subscribers to other searchers' is_target_sensed
        other_searcher_1_config = SEARCHER_CONFIGS[(self.CURRENT_SEARCHER_IDX + 1) % 3]
        other_searcher_2_config =  SEARCHER_CONFIGS[(self.CURRENT_SEARCHER_IDX + 2) % 3]
        rospy.Subscriber(other_searcher_1_config['topic'] + "/target_node_sensed", Bool, self.cb1_target_node_sensed)
        rospy.Subscriber(other_searcher_2_config['topic'] + "/target_node_sensed", Bool, self.cb2_target_node_sensed)

        rospy.Subscriber(other_searcher_1_config['topic'] + "/aoa_strength", Float32, self.cb1_aoa_strength_update)
        rospy.Subscriber(other_searcher_2_config['topic'] + "/aoa_strength", Float32, self.cb2_aoa_strength_update)

        rospy.Subscriber("/group5/start", Bool, self.cb_start_game)
        rospy.Subscriber("/group5/quit", Bool, self.cb_quit_game)
        self.start_game = False
        self.quit_game = False
        
        self.current_state = 'patrolling'   
        print("Initialization successful!")    
        print("First state: " + self.current_state) 

        # In case the callbacks below are never invoked, i.e. other searchers are non-functional somehow.
        self.other_searcher1_target_node_sensed = False
        self.other_searcher1_aoa_strength = MAX_SIGNAL_STRENGTH_VIA_VARIANCE
        self.other_searcher2_target_node_sensed = False
        self.other_searcher2_aoa_strength = MAX_SIGNAL_STRENGTH_VIA_VARIANCE

        self.rerouting_hunter = False
    
    def cb_start_game(self, data):
        self.start_game = data.data
    
    def cb_quit_game(self, data):
        self.quit_game = data.data
    
    def cb1_target_node_sensed(self, node_sensed):
        self.other_searcher1_target_node_sensed = node_sensed.data
    
    def cb2_target_node_sensed(self, node_sensed):
        self.other_searcher2_target_node_sensed = node_sensed.data

    def cb1_aoa_strength_update(self, data):
        print('SIGNAL STRENGTH OF ANOTHER ROBOT UPDATED')
        print(data)
        self.other_searcher1_aoa_strength = data
    
    def cb2_aoa_strength_update(self, data):
        print('SIGNAL STRENGTH OF ANOTHER ROBOT UPDATED')
        print(data)
        self.other_searcher2_aoa_strength = data

    def isDemoted(self):
        if (self.S.aoa_strength > self.other_searcher1_aoa_strength) or (self.S.aoa_strength > self.other_searcher2_aoa_strength):
            print('AGENT IS DEMOTED.')
            return True
        else:
            return False
    

    def loop(self):
        while True:
            # by default, stays within the current state
            next_state = self.current_state 

            if self.quit_game: 
                next_state = 'idle'

            if self.current_state is 'patrolling':
                rospy.loginfo("Patrolling...")

                # Get the locations that robots should go to
                patrolling_location_goal = get_patrolling_locations(SEARCHER_CONFIGS, self.CURRENT_SEARCHER_IDX)

                # Task 2: move robot to location via move_robot_to_waypoint()
                # If the robot is not at the patrol location, go there
                if abs(self.S.get_location()[0] - patrolling_location_goal[0]) > WAYPOINT_THRESHOLD[0] or abs(self.S.get_location()[1] - patrolling_location_goal[1]) > WAYPOINT_THRESHOLD[1]:
                    print("Agent's Location: ", end='')
                    print(self.S.get_location()[0],self.S.get_location()[1])
                    self.S.move_robot_to_waypoint(patrolling_location_goal)
                    print("Waypoint Script is Finished.")

                # Exit conditions
                reached_goal = rospy.wait_for_message(SEARCHER_CONFIGS[self.CURRENT_SEARCHER_IDX]['topic']+'/reached_goal', Bool)

                if reached_goal.data:
                    if self.S.is_target_sensed() or self.other_searcher1_target_node_sensed or self.other_searcher2_target_node_sensed:
                        next_state = 'listening'

            if self.current_state is 'listening':
                # Perform normal state task, be sure to publish aoa data to the other robots
                # AOA angles should be in global frame, not robot frames
                self.S.update_aoa_reading()

                # Exit conditions
                if self.S.aoa_strength < self.other_searcher1_aoa_strength and self.S.aoa_strength < self.other_searcher2_aoa_strength:
                    next_state = 'hunting'
            
            if self.current_state is 'hunting':
                rospy.loginfo("Hunting...")
                
                error_threshold = 10 # degrees
                # first, orient the robot in the proper angle so it is headed at the target
                deg_to_target = self.S.get_location()[2] - self.S.aoa_angle

                is_oriented_to_node = abs(deg_to_target) > error_threshold
                self.rerouting_hunter = self.rerouting_hunter
                obstacle_detected = self.S.obstacle_detected()
                is_demoted = self.isDemoted()
                truthTable = [is_oriented_to_node, self.rerouting_hunter, obstacle_detected, is_demoted]
                print("is_oriented_to_node: " + is_oriented_to_node + 
                    " | self.rerouting_hunter: " + self.rerouting_hunter  +
                    " | obstacle_detected: " + obstacle_detected + 
                    " | is_demoted: " + is_demoted
                )

                # no matter what, if another agent has a better shot at catching the target, then stop and defer
                if is_demoted:
                    self.S.stop_robot()
                    next_state = 'listening'
                    continue

                if truthTable is [1, 0, 0, 0]:
                    self.S.move_robot_in_direction(linear=True,positive=True)
                    continue
                
                if truthTable is [1,1,0,0] or truthTable is [0,1,0,0]:
                    self.rerouting_hunter = False
                    self.S.move_robot_in_direction(linear=True,positive=True)
                
                if truthTable is [1,0,1,0] or truthTable is [0,0,1,0]:
                    self.S.stop_robot()
                    self.rerouting_hunter = True
                
                if truthTable is [0,0,0,0]:
                    self.S.move_robot_in_direction(linear=False, positive=deg_to_target < 0)
                
                if truthTable is [0,1,1,0] or truthTable is [1,1,0]:
                    self.S.move_robot_in_direction(linear=False, positive=deg_to_target < 0)

                ### END NEW LOGIC

                # if self.rerouting_hunter: 
                #     # move angle of robot slightly
                
                #     if not self.S.obstacle_detected():
                #         self.S.move_robot_in_direction(linear=True,positive=True)
                # # robotOrientedToNodeWithinThreshold
                # # self.rerouting_hunter
                # # obstacle_detected
                # # is demoted

                # # first, orient the robot in the proper angle so it is headed at the target
                # deg_to_target = self.S.get_location()[2] - self.S.aoa_angle
                # print("Degrees to target: " + str(deg_to_target))

                # # if robot is not oriented within threshold
                # if abs(deg_to_target) > error_threshold:
                #     self.S.move_robot_in_direction(linear=False, positive=deg_to_target < 0)
                
                # # second, linearly move the robot towards the target
                # else:
                #     #if obstacle, use waypoint script to move around obstacle. Else, move forward
                #     if self.S.obstacle_detected():
                #         self.rerouting_hunter = True
                #         continue
                #         print("Target is likely behind obstacle")
                #     else:
                #         self.S.move_robot_in_direction(linear=True,positive=True)

                # # Exit conditions
                # if self.S.obstacle_detected() or self.isDemoted():
                #     next_state = 'listening'
            
            if self.current_state is 'idle':
                if self.start_game:
                    next_state = 'patrolling'
                    self.start_game = False
            
            if(self.current_state != next_state):
                print("------ Changing state to: " + next_state + ' ------')                    

            self.current_state = next_state

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Get the inputs.')
    parser.add_argument('--robot_index', type=int) # Options: 0,1,2
    args = parser.parse_args()
    F = SearcherFSM(args.robot_index)
    while True: 
        F.loop()