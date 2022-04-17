from searcher import Searcher
import rospy
from std_msgs.msg import Boolean, Float32

NAME = 'searcher1'

class SearcherFSM:
    def __init__(self):
        self.S = Searcher(NAME, '192.168.1.30')

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

    

    def loop(self):
        while True:
            if self.current_state is 'patrolling':
                # Perform normal state task

                # Exit conditions
                if self.S.is_target_sensed() or self.searcher2_target_node_sensed or self.searcher3_target_node_sensed:
                    next_state = 'hunting'

            if self.current_state is 'listening':
                # Perform normal state task
                self.S.update_aoa_reading()

                # Exit conditions
                if self.S.aoa_strength > self.searcher2_aoa_strength and self.S.aoa_strength > self.searcher3_aoa_strength:
                    next_state = 'hunting'
            
            if self.current_state is 'hunting':
                # Perform normal state task

                # Exit conditions
                if self.S.obstacle_detected or self.isDemoted():
                    next_state = 'listening'
            
            if self.current_state is 'idle':
                # Perform normal state task
                if manualResetHit:
                    next_state = 'patrolling'

            self.current_state = next_state
