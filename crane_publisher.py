import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped

crane_x = (6 - 3)*np.random.random_sample() + 3
crane_y = (6 - 3)*np.random.random_sample() + 3
crane_z = (3 - 2)*np.random.random_sample() + 2

def crane_pose():
    pub = rospy.Publisher('crane_pose', PoseStamped, queue_size=10)
    rospy.init_node('crane_pose_publisher', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        crane_pose_msg = PoseStamped()
        crane_pose_msg.header.stamp = rospy.Time.now()
        crane_pose_msg.header.frame_id = "crane"
        crane_pose_msg.pose.position.x = crane_x
        crane_pose_msg.pose.position.y = crane_y
        crane_pose_msg.pose.position.z = crane_z
        crane_pose_msg.pose.orientation.x = 0
        crane_pose_msg.pose.orientation.y = 0
        crane_pose_msg.pose.orientation.z = 0.1613559
        crane_pose_msg.pose.orientation.w = -0.9868963
        pub.publish(crane_pose_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        crane_pose()
    except rospy.ROSInterruptException:
        pass