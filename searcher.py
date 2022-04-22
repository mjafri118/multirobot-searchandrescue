import random
from re import T
import rospy
import math
from wsr_toolbox_cpp.msg import wsr_aoa_array
import numpy as np
from os.path import expanduser
import platform    # For getting the operating system name
import subprocess  # For executing a shell command
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float32
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import LaserScan
from map_to_room_frame import map_to_room_frame, add_barrier_bumper_to_map

STOP_DISTANCE = 0.3
LIDAR_ERROR = 0.05
SAFE_STOP_DISTANCE = STOP_DISTANCE + LIDAR_ERROR

class Searcher: 
    def __init__(self, name, topic, target_ip_address):
        self.name = name
        self.topic = topic
        self.target_ip_address = target_ip_address

        self.aoa_angle = 0 # bounded to [-180, 180]
        self.aoa_strength = 0 # bounded to [0, 1]

        rospy.init_node(name, anonymous=True)
        self.vel_pub = rospy.Publisher("/"+self.topic+"/mobile_base/commands/velocity", Twist, queue_size=1, latch=True)
        self.is_target_sensed_pub = rospy.Publisher("/"+self.topic+"/target_node_sensed", Bool, queue_size=1)
        self.goal_waypoint_pub = rospy.Publisher("/"+self.topic+"/goal_waypoint", Odometry, queue_size=1)
        self.AOA_pub = rospy.Publisher("/"+self.topic+"/aoa_strength", Float32, queue_size=1)
        self.stop_pub = rospy.Publisher("/"+self.topic+"/mobile_base/commands/velocity", Twist, queue_size=1)
        self.request_aoa = rospy.Publisher('/run_test_2',Bool,latch=True, queue_size=1)

    def get_map(self):
        return map_to_room_frame(self.topic)
        
    def filter_map(self):
        return add_barrier_bumper_to_map(self.get_map())
 
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
        self.is_target_sensed_pub.publish(target_sensed)
        return target_sensed
    
    # NOT COMPLETED (random data)
    def update_aoa_reading(self):
        # Test this feature, may need to remove self.stop(robot)
        self.stop_robot()
        print("UPDATING AOA READING")
        self.request_aoa.publish(True)
        aoa_array = rospy.wait_for_message('/wsr_aoa_topic', wsr_aoa_array)
        for tx in aoa_array:
            print("ID: " + tx.id)
            print("Angle: " + str(tx.aoa_azimuth[0]))
            print("Variance: " + str(tx.profile_variance))

            #rospy.init_node('wsr_py_sub_node', anonymous=True)
            self.aoa_angle = tx.aoa_azimuth[0]
            self.aoa_strength = 1-tx.profile_variance # we will choose the highest
        
        # Convert AOA_angle to world frame
        self.aoa_angle = (self.aoa_angle + self.get_location[2]) % 360
        if self.aoa_angle > 180.0 and self.aoa_angle <= 360.0:
            self.aoa_angle = -(360.0 - self.aoa_angle)
        elif self.aoa_angle < -180.0 and self.aoa_angle >= -360.0:
            self.aoa_angle = (360.0 + self.aoa_angle)
        # Publish AOA_strength- MUST HAVE
        self.AOA_pub.publish(self.aoa_strength) # This publishes the AOA to the other robots

    def stop_robot(self):
        # stops robot from moving
        move_cmd = Twist()
        move_cmd.linear.x = 0.0
        move_cmd.angular.z = 0.0
        self.stop_pub.publish(move_cmd)
        return
    
    def obstacle_detected(self):
        lidar_distances = self.get_scan()
        _distance = min(lidar_distances)

        if _distance < SAFE_STOP_DISTANCE and _distance > 0:
            self.stop_robot()
            return True
        return False

    def get_scan(self):
        scan = rospy.wait_for_message(self.topic + '/scan', LaserScan)
        scan_filter = []
       
        samples = len(scan.ranges)  # The number of samples is defined in 
                                    # turtlebot3_<model>.gazebo.xacro file,
                                    # the default is 360.
        samples_view = 120          # 1 <= samples_view <= samples CHANGED FROM 1 TO 180 BY CALEB MOORE
        
        if samples_view > samples:
            samples_view = samples

        if samples_view is 1:
            scan_filter.append(scan.ranges[0])
        else:
            left_lidar_samples_ranges = -(samples_view//2 + samples_view % 2)
            right_lidar_samples_ranges = samples_view//2
            
            left_lidar_samples = scan.ranges[left_lidar_samples_ranges:]
            right_lidar_samples = scan.ranges[:right_lidar_samples_ranges]
            scan_filter.extend(left_lidar_samples + right_lidar_samples)

        for i in range(samples_view):
            if scan_filter[i] == float('Inf') or scan_filter[i] > 2:
                scan_filter[i] = 2.0
            elif math.isnan(scan_filter[i]):
                scan_filter[i] = 0
        
        return scan_filter

    def move_robot_in_direction(self,linear=True,positive=True):
        # note: only making small steps, the FSM will check for obstacles
        # /locobot/mobile_base/commands/velocity geometry_msgs/Twist

        STEP_SIZE_LINEAR = 0.3 # will move these many m/s at a time linearly
        STEP_SIZE_ANGULAR = 0.6 # will move these many rad/s at a time

        move_cmd = Twist()
        # Fill in message details
        if linear and not self.obstacle_detected():
            move_cmd.linear.x = STEP_SIZE_LINEAR
            move_cmd.angular.z = 0.0
        elif not linear:
            move_cmd.linear.x = 0.0
            if positive:
                move_cmd.angular.z = STEP_SIZE_ANGULAR
            else:
                move_cmd.angular.z = -1 * STEP_SIZE_ANGULAR
        # Publish the message
        self.vel_pub.publish(move_cmd)
    
    def move_robot_to_waypoint(self, waypoint):
        # currently requires waypoint.py to be running on each robot
        # give a discrete (x,y,theta) location for robot to travel to with obstacle avoidance
        #  this should just send a publish, i.e. do not make this function wait until reaching the waypoint before completion

        goal_cmd = Odometry()
        goal_cmd.pose.pose.position.x = waypoint[0]
        goal_cmd.pose.pose.position.y = waypoint[1]
        if len(waypoint) == 2:
            goal_cmd.pose.pose.position.z = 0.0
        else:
            goal_cmd.pose.pose.position.z = waypoint[2]
        self.goal_waypoint_pub.publish(goal_cmd)

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
    # print(S.is_moving())
    while True:
        print(S.get_location())
    # print('Is target sensed: ' + 'yes' if S.is_target_sensed() else 'no')
    # S.move_robot_in_direction(linear=True, positive=True)