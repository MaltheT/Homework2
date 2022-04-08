
#!/usr/bin/env python  
import rospy
import math
import tf2_ros
import tf2_geometry_msgs
import geometry_msgs.msg
import tf

def callback(crane_data):    
    br = tf.TransformBroadcaster()
    br.sendTransform(
        (
        crane_data.pose.position.x,
        crane_data.pose.position.y,
        crane_data.pose.position.z),
        (
        crane_data.pose.orientation.x,
        crane_data.pose.orientation.y, 
        crane_data.pose.orientation.z,
        crane_data.pose.orientation.w),
        rospy.Time.now(),
        "inspection",
        "map")
    print("data sent")
    
if __name__ == '__main__':
    rospy.init_node('inspection_to_map_broardcaster')
    crane_subscriber = rospy.Subscriber('/crane_pose', geometry_msgs.msg.PoseStamped, callback) 
    rospy.spin()
