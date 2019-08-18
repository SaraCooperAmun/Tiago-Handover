#By: Sara Cooper
#Runs the transform between /xtion_rg_optical_frame and /aruco_marker_frame as a PoseStamped message to enable object tracking 
#!/usr/bin/env python
import roslib
import rospy
import math
import tf
import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped
if __name__ == '__main__':
    rospy.init_node('aruco_listener')
    listener = tf.TransformListener()
    pub = rospy.Publisher('/xtion_aruco', PoseStamped,queue_size=1)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('/xtion_rgb_optical_frame', '/aruco_marker_frame', rospy.Time(0))
     
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            trans = [0, 0, 0]
            rot = [0, 0, 0, 0]
        aruco_transform = PoseStamped()
        aruco_transform.pose.position.x = trans[0]
        aruco_transform.pose.position.y = trans[1]
        aruco_transform.pose.position.z = trans[2]
        aruco_transform.pose.orientation.x = rot[0]
        aruco_transform.pose.orientation.y = rot[1]
        aruco_transform.pose.orientation.z = rot[2]
        aruco_transform.pose.orientation.w = rot[3]
        pub.publish(aruco_transform)
        rate.sleep()
