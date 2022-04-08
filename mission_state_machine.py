#!/usr/bin/env python  
import rospy
import math
import tf2_ros
import tf2_geometry_msgs
import geometry_msgs.msg
import tf
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from geometry_msgs.msg import TwistStamped, Pose

#class that has robots controlle
class MissionStateMachine:
    #the insitaiser
    def __init__(self):
        # Creates a node with subscriber
        rospy.init_node('mission_state_machine')
        self.x = 0.
        self.y = 0.
        self.z = 0. 
        self.pose_subscriber = rospy.Subscriber('/mavros/global_position/local', Odometry, self.position_callback)
        self.goal_publisher = rospy.Publisher('goal', Pose, queue_size=10)
        
        self.goal_1 = [0, 0, 2]

        msg = Pose()
        msg.position.x = 0.
        msg.position.y = 0.
        msg.position.z = 2.
        while True:
            self.goal_publisher.publish(msg)

    def position_callback(self, data):
        self.x = data.pose.pose.position.x
        self.y = data.pose.pose.position.y
        self.z = data.pose.pose.position.z

   

if __name__ == '__main__':
    try:
        MissionStateMachine()
        rospy.spin()  
    except rospy.ROSInterruptException:
        print ("error!")