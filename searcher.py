import random
import rospy
from wsr_toolbox_cpp.msg import wsr_aoa_array
import numpy as np
from os.path import expanduser
import platform    # For getting the operating system name
import subprocess  # For executing a shell command

STATUS_OPTIONS = ['hunting', 'patrolling', 'listening', 'idle']

class Searcher: 
    def __init__(self, name, target_ip_address):

        # Need something like: rospy.init_node('talker', anonymous=True)
        self.vel_pub = rospy.Publisher("searcher1/locobot/mobile_base/commands/velocity", Twist, queue_size=1)

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
        # Update self.map by subscribing to SLAM map topic
        return None
        
    def is_target_sensed(self):
        # Ping the TX node's ip address, return true/false depending on if ping went through. 
        # use 
        # Option for the number of packets as a function of
        param = '-n' if platform.system().lower()=='windows' else '-c'

        # Building the command. Ex: "ping -c 1 google.com"
        command = ['ping', param, '1', self.target_ip_address]

        target_sensed = subprocess.call(command) == 0
        
        # publish so other searcher FSMs can know when another robot detects the ip.
        rospy.publish('locobotME/target_node_sensed', Boolean, target_sensed) # sync this up with subscribers in FSM inits

        return target_sensed
    
    def update_aoa_reading(self):
        # TODO: actually update AOA reading
        # Make sure angle is relative to 0 deg in environment, not relative to robot heading
        # this will require the robot to be fully stopped, and rotate itself. 
        pub = rospy.Subscriber('wsr_aoa_topic', wsr_aoa_array, self.wsr_cb)
        rospy.init_node('wsr_py_sub_node', anonymous=True)
        # self.aoa_angle = random.uniform(-180,180)
        # self.aoa_strength = random.uniform(0,1)
    
    
    def wsr_cb(self, msg):
        print("######################### Got AOA message ######################")
        for tx in msg.aoa_array:
            print("=========== ID: "+ tx.id +" =============")
            print("TOP N AOA azimuth peak: "+ str(tx.aoa_azimuth))
            print("TOp N AOA elevation peak: "+ str(tx.aoa_elevation))
            print("Profile variance: "+ str(tx.profile_variance))
            print("Profile saved as profile_"+tx.id+".csv")
            self.aoa_angle = tx.aoa_elevation #(Calculation +- with the robot heading angle)
            self.aoa_strength = tx.profile_variance

            homedir = expanduser("~")
            catkin_ws_name = rospy.get_param('~ws_name', 'catkin_ws')
            rootdir = homedir+'/'+catkin_ws_name+"/src/WSR-Toolbox-cpp/debug/"
            aoa_profile = np.asarray(tx.aoa_profile).reshape((tx.azimuth_dim, tx.elevation_dim))
            np.savetxt(rootdir+'/profile_'+tx.id+'.csv', aoa_profile, delimiter=',')


    def stop_robot(self):
        # TODO: stops robot from movement
        self.status = 'idle'
        return
    
    def obstacle_detected(self):
        # TODO, Not Finished
        obstacle = # TODO

        if obstacle:
            while self.vel_pub.get_num_connections() < 1:
                pass
            move_cmd = Twist()
            move_cmd.linear.x = 0.0
            move_cmd.angular.z = 0.0
            self.vel_pub.publish(move_cmd)
            return True
        return False

    def move_robot_in_direction(self,linear=True,positive=True):
        # used in hunting state
        # note: only making small steps, the FSM will check for obstacles
        # /locobot/mobile_base/commands/velocity geometry_msgs/Twist
        # TODO: Figure out how to publish to the right robot
        # Publish message only 1 time per call

        while self.vel_pub.get_num_connections() < 1:
            pass
        move_cmd = Twist()
        # Fill in message details
        if linear:
            move_cmd.linear.x = 0.1
            move_cmd.angular.z = 0.0
        else:
            if positive:
                move_cmd.linear.x = 0.0
                move_cmd.angular.z = 0.1
            else:
                move_cmd.linear.x = 0.0
                move_cmd.angular.z = -0.1
        # Publish the message
        self.vel_pub.publish(move_cmd)
    
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
    
    def update_location(self):
        # Use Subscriber to get current position (x,y,z)
        self.location = None # TODO