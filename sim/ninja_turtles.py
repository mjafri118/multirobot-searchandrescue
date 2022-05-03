'''
Harvard CS 286 Spring 2022
'''

import rospy
import turtlesim.srv
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import pow, atan2, sqrt
'''
Import the modules required for using ROS turtlesim services.
'''
PI = 3.1415926535897



x=0
y=0
z=0
yaw=0

class NinjaTurtles:
    def __init__(self, x, y, name):
         # Creates a node with name 'turtlebot_controller' and make sure it is a
        # unique node (using anonymous=True).
        # rospy.init_node('turtlebot_controller_'+name, anonymous=True)
        # 
        self.x = x
        self.y = y
        self.name = name
        self.pose = Pose()
        self.velocity_publisher = rospy.Publisher('/'+name+'/cmd_vel', Twist, queue_size=10)
        self.pose_subscriber = rospy.Subscriber('/'+name+'/pose', Pose, self.update_pose)
        self.rate = rospy.Rate(10)

    def remove_bot(self,name='turtle1'):
        '''
        Use the turtlesim ROS package service that will remove the default turtle in turtlesim
        '''
        rospy.wait_for_service('kill') 
        killer = rospy.ServiceProxy('kill', turtlesim.srv.Kill)
        killer(name)

    def add_bot(self):
        '''
        Use the turtlesim ROS package service that will spawn a new turtle
        '''
        rospy.wait_for_service('spawn')
        spawner = rospy.ServiceProxy('spawn', turtlesim.srv.Spawn)
        spawner(self.x, self.y, 0, self.name)


    def go_to(self,new_x=0, new_y=0):
        '''
        Use the turtlesim ROS package service that enables a turtle to 
        directly teleport to a new location (new_x,new_y)
        '''
        rospy.wait_for_service(self.name + '/teleport_absolute')
        teleporter = rospy.ServiceProxy(self.name + '/teleport_absolute', turtlesim.srv.TeleportAbsolute)
        teleporter(new_x, new_y, 0)

    def update_pose(self, data):
        """Callback function which is called when a new message of type Pose is received by the subscriber."""
        self.pose = data
        self.pose.x = round(self.pose.x, 4)
        self.pose.y = round(self.pose.y, 4)
        print('called!!!')
        
    def euclidean_distance(self, goal_pose):
        """Euclidean distance between current pose and the goal."""
        return sqrt(pow((goal_pose.x - self.pose.x), 2) + pow((goal_pose.y - self.pose.y), 2))

    def steering_angle(self, goal_pose):
        """Angle which the turtle needs to rotate to align with the goal position."""
        return atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x)
    
    def linear_vel(self, goal_pose, constant=1.5):
        """Velocity as a linear function of the distance to be moved. Velocity = K * Distance"""
        return constant * self.euclidean_distance(goal_pose)

    def angular_vel(self, goal_pose, constant=6):
        """Angular velocity at which the turtle has to rotate to align with goal position"""
        return constant * (self.steering_angle(goal_pose) - self.pose.theta)


    def move2goal(self, goal_x, goal_y, tolerance=0.1):
        """Moves the turtle to the goal."""
        goal_pose = Pose()

        goal_pose.x = goal_x
        # input("Set your x goal: ")
        goal_pose.y = goal_y 
        # input("Set your y goal: ")

        distance_tolerance = tolerance
        # input("Set your tolerance: ")

        vel_msg = Twist()
 
        while self.euclidean_distance(goal_pose) >= distance_tolerance:
            vel_msg.linear.x = self.linear_vel(goal_pose)
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = self.angular_vel(goal_pose)
            self.velocity_publisher.publish(vel_msg)
            self.rate.sleep()
            
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)
        rospy.spin()
    
    def move(self, speed, distance):
        #declare a Twist message to send velocity commands
        velocity_message = Twist()
        #get current location 
        x0=x
        y0=y
        #z0=z;
        #yaw0=yaw;
        velocity_message.linear.x =speed
        distance_moved = 0.0
        loop_rate = rospy.Rate(10) # we publish the velocity at 10 Hz (10 times a second)    
        cmd_vel_topic='/'+self.name+'/cmd_vel'
        velocity_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)

        while True :
                rospy.loginfo("Turtlesim moves forwards")
                velocity_publisher.publish(velocity_message)

                loop_rate.sleep()
                
                #rospy.Duration(1.0)
                
                distance_moved = distance_moved+abs(0.5 * sqrt(((x-x0) ** 2) + ((y-y0) ** 2)))
                print(distance_moved)             
                if  not (distance_moved<distance):
                    rospy.loginfo("reached")
                    break
        
        #finally, stop the robot when the distance is moved
        velocity_message.linear.x =0
        velocity_publisher.publish(velocity_message)


    def rotate(self, speed, angle, clockwise):
        
        velocity_publisher = rospy.Publisher('/'+self.name+'/cmd_vel', Twist, queue_size=10)
        vel_msg = Twist()

        # Receiveing the user's input
        print("Litsening")
        #Converting from angles to radians
        angular_speed = speed*2*PI/360
        relative_angle = angle*2*PI/360

        #We wont use linear components
        vel_msg.linear.x=0
        vel_msg.linear.y=0
        vel_msg.linear.z=0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0

        # Checking if our movement is CW or CCW
        if clockwise:
            vel_msg.angular.z = -abs(angular_speed)
        else:
            vel_msg.angular.z = abs(angular_speed)
        # Setting the current time for distance calculus
        t0 = rospy.Time.now().to_sec()
        current_angle = 0

        while(current_angle < relative_angle):
            velocity_publisher.publish(vel_msg)
            t1 = rospy.Time.now().to_sec()
            current_angle = angular_speed*(t1-t0)


        #Forcing our robot to stop
        vel_msg.angular.z = current_angle
        velocity_publisher.publish(vel_msg)
        
if __name__ == "__main__":
    rospy.init_node("Cowabunga")
    t1 = NinjaTurtles(1,4,'t1')
    # t1.remove_bot('turtle1')
    t1.add_bot()
    t1.go_to(5,5)
    


