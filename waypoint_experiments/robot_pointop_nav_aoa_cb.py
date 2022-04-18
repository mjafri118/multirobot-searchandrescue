import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction , MoveBaseGoal
from tf.transformations import quaternion_from_euler
from std_srvs.srv import Empty
from std_msgs.msg import Float32
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion
from wsr_toolbox_cpp.msg import wsr_aoa_array

msg = """
control your Turtlebot3!
-----------------------
Insert xyz - coordinate.
x : position x (m)
y : position y (m)
z : orientation z (degree: -180 ~ 180)
If you want to close, insert 's'
-----------------------
"""
PI = 3.14

class RobotSLAM_Nav:
    def __init__(self, goal_topic):
        rospy.init_node("move_base_tester")
        self.client = actionlib.SimpleActionClient(goal_topic,MoveBaseAction)
        self.timeout = 60 #secs
        self.step_size = 1.0
        
        #Clear the costmap and rtabmap using rosservice calls.
        rospy.wait_for_service('/locobot/rtabmap/reset')
        reset_map = rospy.ServiceProxy('/locobot/rtabmap/reset', Empty)
        reset_map()

        rospy.wait_for_service('/locobot/move_base/clear_costmaps')
        clear_costmap = rospy.ServiceProxy('/locobot/move_base/clear_costmaps', Empty)
        clear_costmap()

        #Create the actionlib server
        self.client.wait_for_server()

        #Initialize the variable for the goal
        self.goal = MoveBaseGoal()
        self.goal.target_pose.header.frame_id = "map"

        rospy.Subscriber('/locobot/mobile_base/odom', Odometry, self.odom_cb)
        rospy.Subscriber('aoa_topic', Float32, self.aoa_cb_dummy)
        
        rospy.Subscriber('wsr_aoa_topic', wsr_aoa_array, self.wsr_cb)
        
        self.gotAOA = False
        self.current_position = Point()
        self.current_ori = Quaternion()

    def aoa_cb_dummy(self, msg):
        self.move_direction = msg.data 
        self.gotAOA = True
        print("Got new AOA")

    def move(self):

        while True:
            print(msg)
            
            try:
                x, y, z = input("| x | y | z |\n").split()
            except ValueError:
                rospy.loginfo("Shutting Down")
                rospy.signal_shutdown("Shutting Down")
                break

            self.goal.target_pose.header.stamp = rospy.Time.now()
            self.goal.target_pose.pose.position.x = float(x)
            self.goal.target_pose.pose.position.y = float(y)
            
            q = quaternion_from_euler(0,0,float(z))

            self.goal.target_pose.pose.orientation.x = q.x
            self.goal.target_pose.pose.orientation.y = q.y
            self.goal.target_pose.pose.orientation.z = q.z
            self.goal.target_pose.pose.orientation.w = q.w
            
            rospy.loginfo("Attempting to move to the goal")
            self.client.send_goal(self.goal)
            wait=self.client.wait_for_result(rospy.Duration(self.timeout))

            if not wait:
                rospy.loginfo("Timed-out after failing to reach the goal.")
                self.client.cancel_goal()
                rospy.loginfo("Please provide a new goal position")
            else:
                rospy.loginfo("Reached goal successfully")

    def wsr_cb(self,msg):
        print("######################### Got message ######################")
        for tx in msg.aoa_array:
            print("=========== ID: "+ tx.id +" =============")
            print("TOP N AOA azimuth peak: "+ str(tx.aoa_azimuth))
            print("TOp N AOA elevation peak: "+ str(tx.aoa_elevation))
            print("Profile variance: "+ str(tx.profile_variance))
            self.move_direction = tx.aoa_azimuth[0]
            self.gotAOA = True
            print("Got new AOA")
            #print("Profile saved as profile_"+tx.id+".csv")

            #homedir = expanduser("~")
            #catkin_ws_name = rospy.get_param('~ws_name', 'cs286_hack_ws')
            #rootdir = homedir+'/'+catkin_ws_name+"/src/WSR-Toolbox-cpp/debug/"
            #aoa_profile = np.asarray(tx.aoa_profile).reshape((tx.azimuth_dim, tx.elevation_dim))
            #np.savetxt(rootdir+'/profile_'+tx.id+'.csv', aoa_profile, delimiter=',')


    def odom_cb(self,msg):
        self.current_position = msg.pose.pose.position
        self.current_ori.x, self.current_ori.y, self.current_ori.z, self.current_ori.w = msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w

    def move_along_direction(self):
        rospy.loginfo("Waiting for AOA...")
        while not rospy.is_shutdown():    
            if(self.gotAOA):
                x = self.current_position.x + self.step_size*math.cos(self.move_direction*PI/180)
                y = self.current_position.y + self.step_size*math.sin(self.move_direction*PI/180)

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

                self.gotAOA = False

if __name__=='__main__':
    obj = RobotSLAM_Nav('/locobot/move_base')
    
    #obj.move()
    obj.move_along_direction()
