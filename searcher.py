import random
import rospy
from wsr_toolbox_cpp.msg import wsr_aoa_array
import numpy as np
from os.path import expanduser
import platform    # For getting the operating system name
import subprocess  # For executing a shell command
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion


class Searcher: 
    def __init__(self, name, topic, target_ip_address):
        self.name = name
        self.topic = topic
        self.target_ip_address = target_ip_address

        self.aoa_angle = 0 # bounded to [-180, 180]
        self.aoa_strength = 0 # bounded to [0, 1]

        self.map = None # don't access this variable. always call get_map() for most up-to-date map.

        rospy.init_node(name, anonymous=True)
        self.vel_pub = rospy.Publisher(self.topic + "/mobile_base/commands/velocity", Twist, queue_size=1, latch=True)
        # self.vel_pub = rospy.Publisher("locobot/mobile_base/commands/velocity", Twist, queue_size=1)

    # NOT COMPLETED
    def get_map(self):
        # TODO
        # Update self.map by subscribing to SLAM map topic
        return None

    # NOT COMPLETED    
    def is_target_sensed(self):
        # Ping the TX node's ip address, return true/false depending on if ping went through. 
        # use 
        # Option for the number of packets as a function of
        param = '-n' if platform.system().lower()=='windows' else '-c'

        # Building the command. Ex: "ping -c 1 google.com"
        command = ['ping', param, '1', self.target_ip_address]

        target_sensed = subprocess.call(command, stdout=subprocess.PIPE) == 0
        
        # publish so other searcher FSMs can know when another robot detects the ip.
        # rospy.publish('locobotME/target_node_sensed', Boolean, target_sensed) # sync this up with subscribers in FSM inits

        return target_sensed
    
    # NOT COMPLETED
    def update_aoa_reading(self):
        # TODO: actually update AOA reading
        # Make sure angle is relative to 0 deg in environment, not relative to robot heading
        # this will require the robot to be fully stopped, and rotate itself. 
        pub = rospy.Subscriber('wsr_aoa_topic', wsr_aoa_array, self.wsr_cb)
        rospy.init_node('wsr_py_sub_node', anonymous=True)
        # self.aoa_angle = random.uniform(-180,180)
        # self.aoa_strength = random.uniform(0,1)
    
    # NOT COMPLETED
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

    # NOTE COMPLETED
    def stop_robot(self):
        # TODO: stops robot from movement
        self.status = 'idle'
        return
    
    # NOT COMPLETED
    def obstacle_detected(self):
        # TODO, Not Finished
        obstacle = True

        if obstacle:
            while self.vel_pub.get_num_connections() < 1:
                pass
            move_cmd = Twist()
            move_cmd.linear.x = 0.0
            move_cmd.angular.z = 0.0
            self.vel_pub.publish(move_cmd)
            return True
        return False

    # NOT COMPLETED
    def move_robot_in_direction(self,linear=True,positive=True):
        # note: only making small steps, the FSM will check for obstacles
        # /locobot/mobile_base/commands/velocity geometry_msgs/Twist

        STEP_SIZE_LINEAR = 1.0 # will move these many m/s at a time linearly
        STEP_SIZE_ANGULAR = 1.0 # will move these many rad/s at a time

        move_cmd = Twist()
        # Fill in message details
        if linear:
            move_cmd.linear.x = STEP_SIZE_LINEAR
            move_cmd.angular.z = 0.0
        else:
            move_cmd.linear.x = 0.0
            if positive:
                move_cmd.angular.z = STEP_SIZE_ANGULAR
            else:
                move_cmd.angular.z = -1 * STEP_SIZE_ANGULAR
        # Publish the message
        print(move_cmd)
        self.vel_pub.publish(move_cmd)
    
    # NOT COMPLETED
    def move_robot_to_waypoint(self, waypoint):
        # TODO: give a discrete (x,y) location for robot to travel to with obstacle avoidance
        #  this should just send a publish, i.e. do not make this function wait until reaching the waypoint before completion
        pass

    def is_moving(self):
        # inferred from reading Odometry data from robot.
        # returns true or false
        topic_response = rospy.wait_for_message(self.topic + '/mobile_base/odom', Odometry)
        twistLinearX_moving = abs(topic_response.twist.twist.linear.x) > 0
        twistAngularZ_moving = abs(topic_response.twist.twist.angular.z) > 0.05 
        return twistAngularZ_moving or twistLinearX_moving
    
    def get_location(self):
        # Uses one-time subscriber to get current position (x,y,w) relative to where the launch file of robot was invoked
        # note w is in degrees
        
        topic_response = rospy.wait_for_message(self.topic + '/mobile_base/odom', Odometry)
        orientation = (topic_response.pose.pose.orientation.x, 
            topic_response.pose.pose.orientation.y, 
            topic_response.pose.pose.orientation.z, 
            topic_response.pose.pose.orientation.w)
        return (topic_response.pose.pose.position.x, topic_response.pose.pose.position.y, np.rad2deg(euler_from_quaternion(orientation)[2]))

    
if __name__ == "__main__":
    S = Searcher('searcher1', 'locobot', '192.168.1.30')
    print(S.is_moving())
    print(S.get_location())
    print('Is target sensed: ' + 'yes' if S.is_target_sensed() else 'no')
    # S.move_robot_in_direction(linear=True, positive=True)