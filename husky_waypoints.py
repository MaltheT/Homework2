#!/usr/bin/env python

#imports for robot controlles
import rospy
import csv 
from turtlesim.msg import Pose
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry
import math

#the giving function for quaternion to euler
def quaternion_to_euler(x, y, z, w):
        
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        X = math.degrees(math.atan2(t0, t1))
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        Y = math.degrees(math.asin(t2))
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        Z = math.degrees(math.atan2(t3, t4))
        X = math.atan2(t0, t1)
        Y = math.asin(t2)
        Z = math.atan2(t3, t4)
        return X, Y, Z

#class that has robots controlle
class RobotController:
    #the insitaiser
    def __init__(self):
        #rospy sub for the modelstate topic
        self.pose_subscriber = rospy.Subscriber('gazebo/model_states', ModelStates, self.callback)
        
        #calls the function writeToCsv after 70 seconds once
        rospy.Timer(rospy.Duration(70), self.writeToCsv, oneshot=True)
        #data list for the visuale data
        self.x_data = []
        self.y_data = []
        self.counter = 5

        #the list with the waypoints the robot has to go to
        self.waypoints_x = [11.5, 11.3, 3.4, 0.0]
        self.waypoints_y = [-0.3, 5.1, 4.9, 0.0]
        self.next_waypoint = 0
        
        #current goal the for the robot
        self.xr_goal = self.waypoints_x[self.next_waypoint]
        self.yr_goal = self.waypoints_y[self.next_waypoint]

        #gian values for the controller
        self.linear_gain = 0.2
        self.angular_gain = 10

    def callback(self, data):
        #rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.pose[1].orientation)

        #getting the angle of the robot in z axie
        _, _, theta = quaternion_to_euler(
        data.pose[2].orientation.x,
        data.pose[2].orientation.y,
        data.pose[2].orientation.z,
        data.pose[2].orientation.w)

        #the cordinates for the robot on x and y axies
        xr = data.pose[2].position.x
        yr = data.pose[2].position.y

        #is for the data collection. It collects eary 5th data point 
        if(self.counter == 5):
            self.x_data.append(xr)
            self.y_data.append(yr)
            self.counter = 0
        else:
            self.counter +=1
        
        #the call for the controller function
        self.robot_controller(theta, xr, yr)
    


    def robot_controller(self, theta, xr, yr):
        #creats a publiser to the twist topic and a twist massage
        pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        msg = Twist()

        #calculates the distens to the goal form the robot
        distance = math.sqrt((xr - self.xr_goal)**2 + (yr - self.yr_goal)**2)
        
        #calculates the angle to the goal form the robot trajectory
        desired_angle = math.atan2(self.yr_goal-yr, self.xr_goal-xr) - theta
        
        # code inspiered by https://wiki.nps.edu/display/RC/Husky+Control+in+Gazebo
        bound = math.atan2(math.sin(desired_angle),math.cos(desired_angle))
        w = min(0.5 , max(-0.5, self.angular_gain*bound))

        # checks if the robot is not with in the threshold
        if(distance > 1.5): # 1.5 threshold
            msg.linear.x = distance * self.linear_gain
            msg.angular.z = w
        # then it check if there are more waypoint and then set the next as target
        elif (self.next_waypoint < 3):
            self.next_waypoint += 1
            self.xr_goal = self.waypoints_x[self.next_waypoint]
            self.yr_goal = self.waypoints_y[self.next_waypoint]
        # else stops the robot
        else:
            msg.linear.x = 0
            msg.angular.z = 0

        # sends the message to twist topic
        pub.publish(msg)

    #creats our data files for the polts of the robot movement
    def writeToCsv(self, event):
        with open('trajectory_x.csv', 'w') as file:
            writer = csv.writer(file) 
            writer.writerow(self.x_data)
            file.close()

        with open('trajectory_y.csv', 'w') as file:
            writer = csv.writer(file)
            writer.writerow(self.y_data)
            file.close()

# main that creats the class and spins
if __name__ == '__main__':
    try:
        rospy.init_node('robot_controller')
        RobotController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

