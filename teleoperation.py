#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
import numpy as np
from math import pi
from scipy.spatial.transform import Rotation as R

# Replace this with code to get the pose from your device
def get_teleoperation_pose():

    Rot = np.array([
        [1, 0, 0],
        [0, np.cos(pi), -np.sin(pi)],
        [0, np.sin(pi), np.cos(pi)]
        ])
    
    qx, qy, qz, qw = (R.from_dcm(Rot)).as_quat()

    # Example pose, replace with your teleoperation device's logic
    pose = PoseStamped()
    pose.header.stamp = rospy.Time.now()
    pose.header.frame_id = "base_link"  # Change to the appropriate frame
    pose.pose.position.x = 0.30
    pose.pose.position.y = 0.40
    pose.pose.position.z = 0.40
    pose.pose.orientation.x = qx
    pose.pose.orientation.y = qy
    pose.pose.orientation.z = qz
    pose.pose.orientation.w = qw
    return pose

def teleoperation_publisher():
    rospy.init_node('teleoperation_publisher', anonymous=True)
    pub = rospy.Publisher('/desired_pose', PoseStamped, queue_size=10)
    rate = rospy.Rate(50)  # Publish at 500 Hz

    while not rospy.is_shutdown():
        pose = get_teleoperation_pose()  # Get the desired pose from your device
        pub.publish(pose)  # Publish it to /desired_pose
        rate.sleep()

if __name__ == '__main__':
    try:
        teleoperation_publisher()
    except rospy.ROSInterruptException:
        pass
