# Terminal command to be run on robot: python waypoint.py --robot_name locobot3
# Terminal command to be run on robot: python waypoint.py --robot_name locobot4
# Terminal command to be run on robot: python waypoint.py --robot_name locobot5

from termios import TIOCPKT_DOSTOP
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction , MoveBaseGoal
from tf.transformations import quaternion_from_euler
from std_srvs.srv import Empty
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from geometry_msgs.msg import Point, Quaternion
import argparse


class RobotSLAM_Nav:
    def __init__(self, robot_name):
        rospy.init_node("move_base_tester", anonymous=True)
        self.client = actionlib.SimpleActionClient('/'+robot_name+'/move_base',MoveBaseAction)
        self.timeout = 60 #secs
        self.step_size = 1.0
        self.reached_goal = rospy.Publisher(robot_name+'/reached_goal',Bool,latch=True, queue_size=1)
        
        #Clear the costmap and rtabmap using rosservice calls.
        print("Resetting rtabmap")
        rospy.wait_for_service('/'+robot_name+'/rtabmap/reset')
        reset_map = rospy.ServiceProxy('/'+robot_name+'/rtabmap/reset', Empty)
        reset_map()
        print("Rtab map reset done")

        print("Clearing cost map")
        rospy.wait_for_service('/'+robot_name+'/move_base/clear_costmaps')
        clear_costmap = rospy.ServiceProxy('/'+robot_name+'/move_base/clear_costmaps', Empty)
        clear_costmap()
        print("Cost map cleared")

        #Create the actionlib server
        print("Waiting for action lib server")
        self.client.wait_for_server()

        #Initialize the variable for the goal
        self.goal = MoveBaseGoal()
        self.goal.target_pose.header.frame_id = robot_name+"/map"


        # This subscriber listens for goals to go to on TODO_topic
        rospy.Subscriber('/'+robot_name+'/goal_waypoint', Odometry, self.goal_callback)
        self.gotGOAL = False
        self.current_position = Point()
        self.current_ori = Quaternion()

    def goal_callback(self, msg):
        self.goal_x = msg.pose.pose.position.x
        self.goal_y = msg.pose.pose.position.y
        self.goal_z = msg.pose.pose.orientation.z
        self.gotGOAL = True
        print("Got new goal")

    def move(self):
        rospy.loginfo("Waiting for Goal...")
        while not rospy.is_shutdown():    
            if(self.gotGOAL):
                # Add your code here to update x and y based on the goal
                x = self.goal_x
                y = self.goal_y
                z = self.goal_z
                print("Moving to next location x = ",x, ", y =",y)

                self.goal.target_pose.header.stamp = rospy.Time.now()
                self.goal.target_pose.pose.position.x = float(x)
                self.goal.target_pose.pose.position.y = float(y)
            
                q = quaternion_from_euler(0,0,float(z))

                self.goal.target_pose.pose.orientation.x = q[0]
                self.goal.target_pose.pose.orientation.y = q[1]
                self.goal.target_pose.pose.orientation.z = q[2]
                self.goal.target_pose.pose.orientation.w = q[3]

                rospy.loginfo("Attempting to move to the goal")
                self.reached_goal.publish(False)
                self.client.send_goal(self.goal)
                wait=self.client.wait_for_result(rospy.Duration(self.timeout))

                if not wait:
                    rospy.loginfo("Timed-out after failing to reach the goal.")
                    self.client.cancel_goal()
                    self.gotGOAL = False
                    rospy.loginfo("Please provide a new goal position")
                else:
                    rospy.loginfo("Reached goal successfully")
                    self.reached_goal.publish(True)
                    self.gotGOAL = False

if __name__=='__main__':
    parser = argparse.ArgumentParser(description='Get the inputs.')
    parser.add_argument('--robot_name', type=str)
    args = parser.parse_args()
    obj = RobotSLAM_Nav(args.robot_name)
    obj.move()

    #robot_name = 'locobot4'
    #obj = RobotSLAM_Nav(robot_name)
    #obj.move()
