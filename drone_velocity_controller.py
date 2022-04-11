#!/usr/bin/env python

#imports for robot controlles
from dis import dis
import rospy
import csv 
from std_msgs.msg import String
from geometry_msgs.msg import TwistStamped, Pose
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry
import math
from mavros_msgs.srv import SetMode
from mavros_msgs.srv import CommandBool
from std_msgs.msg import Float32
import numpy as np


    


class DroneController:
    def __init__(self):
        rospy.init_node('drone_velocity_controller') 
    
        self.x = 0.
        self.y = 0.
        self.z = 0.

        self.goal_subscriber = rospy.Subscriber('/goal', Pose, self.goal_callback)
        self.pose_subscriber = rospy.Subscriber('/mavros/global_position/local', Odometry, self.position_callback)
        self.velocity_pub = rospy.Publisher('mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)
        rospy.Timer(rospy.Duration(60), self.writeToCsv, oneshot=True)

        # initial goal for hovering
        self.x_goal = 0.
        self.y_goal = 0.
        self.z_goal = 2.

        self.e_x = 0.
        self.e_y = 0.
        self.e_z = 0.

        self.e_x_prev = 0.
        self.e_y_prev = 0.
        self.e_z_prev = 0.

        self.e_x_acc = 0.
        self.e_y_acc = 0.
        self.e_z_acc = 0.

        self.kp = 0.5
        self.ki = 0.0
        self.kd = 0.1

        # logging for plotting
        self.x_log = []
        self.y_log = []
        self.z_log = []
        self.dist_log = []

        # putting control callback on timer
        self.timer = rospy.Timer(rospy.Duration(0.1), self.control_callback)

        # Output a few position control setpoint so px4 allow to change into "Offboard" flight mode (i.e. to allow control code running from a companion computer)
        # Note: this can be any control setpoint (e.g. velocities, attitude , etc.)
        r = rospy.Rate(50)
        for i in range (110):
            self.PID()
            r.sleep()

        # Now we can initialize the drone
        self.initialize_drone()

        
    def control_callback(self, event):
        #print ('Timer called at ' + str(event.current_real))  
        self.PID()

    def position_callback(self, data):
        self.x = data.pose.pose.position.x
        self.y = data.pose.pose.position.y
        self.z = data.pose.pose.position.z

        self.x_log.append(self.x)
        self.y_log.append(self.y)
        self.z_log.append(self.z)
        dist = np.linalg.norm(np.array([self.x, self.y, self.z]) - np.array([self.x_goal, self.y_goal, self.z_goal]))
        self.dist_log.append(dist)

    def goal_callback(self, data):
        self.x_goal = data.position.x
        self.y_goal = data.position.y
        self.z_goal = data.position.z


    def PID(self):
        self.e_x = self.x_goal - self.x
        self.e_y = self.y_goal - self.y
        self.e_z = self.z_goal - self.z

        # only accumilate error if we're in the air
        if (self.z > 0.2):
            self.e_x_acc += self.e_x
            self.e_y_acc += self.e_y
            self.e_z_acc += self.e_z
        else:
            self.e_x_acc += 0
            self.e_y_acc += 0
            self.e_z_acc += 0

        msg = TwistStamped()
        msg.header.stamp = rospy.Time.now()
        msg.twist.linear.x = self.kp * self.e_x + self.ki * self.e_x_acc + self.kd * (self.e_x - self.e_x_prev)
        msg.twist.linear.y = self.kp * self.e_y + self.ki * self.e_y_acc + self.kd * (self.e_y - self.e_y_prev)
        msg.twist.linear.z = self.kp * self.e_z + self.ki * self.e_z_acc + self.kd * (self.e_z - self.e_z_prev)
        
        self.e_x_prev = self.e_x
        self.e_y_prev = self.e_y
        self.e_z_prev = self.e_z
    
        self.velocity_pub.publish(msg)

    def initialize_drone(self):

        # Set Mode
        print ("\nSetting Mode")
        rospy.wait_for_service('/mavros/set_mode')
        try:
            change_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)
            response = change_mode(custom_mode="OFFBOARD")
            rospy.loginfo(response)
        except rospy.ServiceException as e:
            print("Set mode failed: %s" %e)

        # Arm
        print ("\nArming")
        rospy.wait_for_service('/mavros/cmd/arming')
        try:
            arming_cl = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
            response = arming_cl(value = True)
            rospy.loginfo(response)
        except rospy.ServiceException as e:
            print("Arming failed: %s" %e)

    def writeToCsv(self, event):
        with open('x.csv', 'w') as file:
            writer = csv.writer(file) 
            writer.writerow(self.x_log)
            file.close()

        with open('y.csv', 'w') as file:
            writer = csv.writer(file)
            writer.writerow(self.y_log)
            file.close()

        with open('z.csv', 'w') as file:
            writer = csv.writer(file)
            writer.writerow(self.z_log)
            file.close()

        with open('distance.csv', 'w') as file:
            writer = csv.writer(file)
            writer.writerow(self.dist_log)
            file.close()
        print("data written to csv")

if __name__ == '__main__':
    try:
        DroneController()
        rospy.spin()  
    except rospy.ROSInterruptException:
        print ("error!")