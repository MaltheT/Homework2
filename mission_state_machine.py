#!/usr/bin/env python  
from multiprocessing.connection import Listener
import rospy
import math
import tf2_ros
import tf2_geometry_msgs
import geometry_msgs.msg
import tf
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from geometry_msgs.msg import Pose
import tf2_geometry_msgs



#class that has robots controlle
class MissionStateMachine:
    #the insitaiser
    def __init__(self):
        # Creates a node with subscriber
        rospy.init_node('mission_state_machine')
        self.x = 0.
        self.y = 0.
        self.z = 0. 
        self.threshold = 0.2

        self.pose_subscriber = rospy.Subscriber('/mavros/global_position/local', Odometry, self.position_callback)
        self.goal_publisher = rospy.Publisher('goal', Pose, queue_size=10)

        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)

        self.waypoints = [[2, 2, 0], [2, -2, 0], [-2, -2, 0], [-2, 2, 0], [2, 2, 0], [0, 0, 2], [0, 0, 0]]
        self.curr_waypoint_idx = 0
        

    def position_callback(self, data):
        self.x = data.pose.pose.position.x
        self.y = data.pose.pose.position.y
        self.z = data.pose.pose.position.z


        mission_pose = Pose()
        mission_pose.position.x = self.waypoints[self.curr_waypoint_idx][0]
        mission_pose.position.y = self.waypoints[self.curr_waypoint_idx][1]
        mission_pose.position.z = self.waypoints[self.curr_waypoint_idx][2]

        # only convert first 4 points, last two are already in map fram
        if(self.curr_waypoint_idx < len(self.waypoints)-2):
            # converting coordinates from inspection fram to map frame
            mission_pose = self.transform_pose(mission_pose, "inspection", "map")
            print(mission_pose)

        self.goal_publisher.publish(mission_pose)

        #if x y z values are within 
        if((abs(self.x - mission_pose.position.x) < self.threshold) and 
           (abs(self.y - mission_pose.position.y) < self.threshold) and 
           (abs(self.z - mission_pose.position.z) < self.threshold) and
           (self.curr_waypoint_idx < len(self.waypoints))): 
           self.curr_waypoint_idx += 1
        


    # reference: https://answers.ros.org/question/323075/transform-the-coordinate-frame-of-a-pose-from-one-fixed-frame-to-another/
    def transform_pose(self, input_pose, from_frame, to_frame):
        pose_stamped = tf2_geometry_msgs.PoseStamped()
        pose_stamped.pose = input_pose
        pose_stamped.header.frame_id = from_frame
        pose_stamped.header.stamp = rospy.Time(0)

        try:
            # ** It is important to wait for the listener to start listening. Hence the rospy.Duration(1)
            output_pose_stamped = self.tf_buffer.transform(pose_stamped, to_frame, rospy.Duration(1))
            return output_pose_stamped.pose

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            raise

if __name__ == '__main__':
    try:
        MissionStateMachine()
        rospy.spin()  
    except rospy.ROSInterruptException:
        print ("error!")