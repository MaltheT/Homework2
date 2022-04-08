#!/usr/bin/env python

#imports for robot controlles
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
class DroneController:
    #the insitaiser
    def __init__(self):
        # Creates a node with subscriber
        rospy.init_node('drone_velocity_controller') 
    
        self.x = 0.
        self.y = 0.
        self.z = 0.

        self.goal_subscriber = rospy.Subscriber('/goal', Pose, self.goal_callback)
        self.pose_subscriber = rospy.Subscriber('/mavros/global_position/local', Odometry, self.position_callback)
        self.velocity_pub = rospy.Publisher('mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)

        self.x_goal = 0.
        self.y_goal = 0.
        self.z_goal = 0.

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

        self.threshold = 0.1

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

    def goal_callback(self, data):
        self.x_goal = data.position.x
        self.y_goal = data.position.y
        self.z_goal = data.position.z


    def PID(self):
        self.e_x = self.x_goal - self.x
        self.e_y = self.y_goal - self.y
        self.e_z = self.z_goal - self.z

        self.e_x_acc += self.e_x
        self.e_y_acc += self.e_y
        self.e_z_acc += self.e_y

        msg = TwistStamped()
        msg.header.stamp = rospy.Time.now()
        msg.twist.linear.x = self.kp * self.e_x + self.ki * self.e_x_acc + self.kd * (self.e_x - self.e_x_prev)
        msg.twist.linear.y = self.kp * self.e_y + self.ki * self.e_y_acc + self.kd * (self.e_y - self.e_y_prev)
        msg.twist.linear.z = self.kp * self.e_z + self.ki * self.e_z_acc + self.kd * (self.e_z - self.e_z_prev)
        
        self.e_x_prev = self.e_x
        self.e_y_prev = self.e_y
        self.e_z_prev = self.e_z

        print(self.z_goal)
    
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


if __name__ == '__main__':
    try:
        DroneController()
        rospy.spin()  
    except rospy.ROSInterruptException:
        print ("error!")