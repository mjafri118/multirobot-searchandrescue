from searcher import Searcher
from patrol import get_patrolling_locations
import rospy
from std_msgs.msg import Boolean, Float32

SEARCHER_CONFIGS = [
    {
        "name": 'searcher1',
        "topic": '/locobot1'
    },
    {
        "name": 'searcher1',
        "topic": '/locobot2'
    },
    {
        "name": 'searcher1',
        "topic": '/locobot3'
    }
]

CURRENT_SEARCHER_IDX = 0

TARGET_NODE_IP = '192.168.1.30'



class SearcherFSM:
    def __init__(self):
        self.S = Searcher(SEARCHER_CONFIGS[CURRENT_SEARCHER_IDX], TARGET_NODE_IP)

        # setup callback subscribers to other searchers' is_target_sensed
        rospy.Subscriber("searcher2/target_node_sensed", Boolean, self.cb2_target_node_sensed)
        rospy.Subscriber("searcher3/target_node_sensed", Boolean, self.cb3_target_node_sensed)

        rospy.Subscriber("searcher2/aoa_strength", Float32, self.cb2_aoa_strength_update)
        rospy.Subscriber("searcher3/aoa_strength", Float32, self.cb3_aoa_strength_update)
        
        self.current_state = 'patrolling'

        
    
    def cb2_target_node_sensed(self, name, data):
        self.searcher2_target_node_sensed = data
    
    def cb3_target_node_sensed(self, name, data):
        self.searcher3_target_node_sensed = data

    def cb2_aoa_strength_update(self, name, data):
        self.searcher2_aoa_strength = data
    
    def cb3_aoa_strength_update(self, name, data):
        self.searcher3_aoa_strength = data

    def isDemoted(self):
        if (self.S.aoa_strength < self.searcher2_aoa_strength) or (self.S.aoa_strength < self.searcher3_aoa_strength):
            return True
        else:
            return False
    

    def loop(self):
        while True:
            if self.current_state is 'patrolling':
                # Perform normal state task             
                # Get the locations that robots should go to, given
                # Task 2: move robot to location via move_robot_to_waypoint()
                if not self.S.is_moving():
                    patrolling_location_goal = get_patrolling_locations(SEARCHER_CONFIGS, CURRENT_SEARCHER_IDX)
                    self.S.move_robot_to_waypoint(patrolling_location_goal)
                next_state = 'patrolling' # Because we don't have a next_state yet

                # Exit conditions
                if self.S.is_target_sensed() or self.searcher2_target_node_sensed or self.searcher3_target_node_sensed:
                    next_state = 'listening'

            if self.current_state is 'listening':
                # Perform normal state task, be sure to publish aoa data to the other robots
                # AOA angles should be in global frame, not robot frames
                self.S.update_aoa_reading()

                # Exit conditions
                if self.S.aoa_strength > self.searcher2_aoa_strength and self.S.aoa_strength > self.searcher3_aoa_strength:
                    next_state = 'hunting'
            
            if self.current_state is 'hunting':
                # Perform normal state task
                error_threshold = 5 # degrees
                # first, orient the robot in the proper angle so it is headed at the target
                rad_to_target = self.S.get_location()[2] - self.S.aoa_angle
                if abs(rad_to_target) > error_threshold:
                    self.S.move_robot_in_direction(linear=False, positive=rad_to_target > 0)
                
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

            self.current_state = next_state
