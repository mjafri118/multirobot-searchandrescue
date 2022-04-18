import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction , MoveBaseGoal
from tf.transformations import quaternion_from_euler
from std_srvs.srv import Empty
from std_msgs.msg import Float32
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion
import argparse

# PI = 3.14

# if this works w/o position topic specified, remove mention of it from class instatntiation + all of code
class RobotSLAM_Nav:
    def __init__(self, goal_topic, position_topic,bot):
        rospy.init_node("move_base_tester")
        self.client = actionlib.SimpleActionClient(goal_topic,MoveBaseAction)
        self.timeout = 60 #secs
        self.step_size = 1.0
        
        if(bot == 1):
            #Clear the costmap and rtabmap using rosservice calls.
            rospy.wait_for_service('/locobot4/rtabmap/reset')
            reset_map = rospy.ServiceProxy('/locobot4/rtabmap/reset', Empty)
            reset_map()

            rospy.wait_for_service('/locobot4/move_base/clear_costmaps')
            clear_costmap = rospy.ServiceProxy('/locobot4/move_base/clear_costmaps', Empty)
            clear_costmap()
        else:
            #Clear the costmap and rtabmap using rosservice calls.
            rospy.wait_for_service('/rtabmap/reset')
            reset_map = rospy.ServiceProxy('/rtabmap/reset', Empty)
            reset_map()

            rospy.wait_for_service('/move_base/clear_costmaps')
            clear_costmap = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
            clear_costmap()


        #Create the actionlib server
        self.client.wait_for_server()

        #Initialize the variable for the goal
        self.goal = MoveBaseGoal()
        self.goal.target_pose.header.frame_id = "map"

        # rospy.Subscriber(position_topic, Odometry, self.odom_cb)
        rospy.Subscriber('locobot4/command/move_to_waypoint', Odometry, self.new_waypoint_goal_cb)
        self.gotGoal = False
        self.current_position = Point()
        self.current_ori = Quaternion()

    def new_waypoint_goal_cb(self, msg):
        self.goalX = msg.pose.pose.position.x
        self.goalY = msg.pose.pose.position.y
        self.gotGoal = True
        print("Got new Goal!")


    # def odom_cb(self,msg):
    #     #Update this callback function to get the positions of the robot from its odometery.
    #     #Use the variables self.current_position and self.current_ori to store the position and orientation values.
    #     self.current_position.x, self.current_position.y, self.current_position.z = msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z
    #     self.current_ori.x, self.current_ori.y, self.current_ori.z, self.current_ori.w,  = msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w  
    
    def move_along_direction(self):
        rospy.loginfo("Waiting for Goal from locobot4/command/move_to_waypoint...")
        while not rospy.is_shutdown():    
            if(self.gotGoal):
                # Add your code here to update x and y based on the AOA direction and step-size
                x = self.goalX
                y = self.goalY

                print("Moving to next location x = ",x, ", y =",y)
                
                self.goal.target_pose.header.stamp = rospy.Time.now()
                self.goal.target_pose.pose.position.x = x
                self.goal.target_pose.pose.position.y = y

                self.goal.target_pose.pose.orientation.x = self.current_ori.x
                self.goal.target_pose.pose.orientation.y = self.current_ori.y
                self.goal.target_pose.pose.orientation.z = self.current_ori.z
                self.goal.target_pose.pose.orientation.w = self.current_ori.w
                
                rospy.loginfo("Attempting to move to the goal")
                self.client.send_goal(self.goal)
                wait=self.client.wait_for_result(rospy.Duration(self.timeout))

                if not wait:
                    rospy.loginfo("Timed-out after failing to reach the goal.")
                    self.client.cancel_goal()
                    rospy.loginfo("Please provide a new goal position")
                else:
                    rospy.loginfo("Reached goal successfully")

                self.gotGoal = False

if __name__=='__main__':
    parser = argparse.ArgumentParser(description='Get the inputs.')
    parser.add_argument('--goal_topic', type=str)
    parser.add_argument('--position_topic', type=str)
    parser.add_argument('--bot', type=int)
    args = parser.parse_args()
    obj = RobotSLAM_Nav(args.goal_topic, args.position_topic, args.bot)
    
    # obj.move()
    obj.move_along_direction()
