#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped

# Replace this with code to get the pose from your device
def get_teleoperation_pose():
    # Example pose, replace with your teleoperation device's logic
    pose = PoseStamped()
    pose.header.stamp = rospy.Time.now()
    pose.header.frame_id = "base"  # Change to the appropriate frame
    pose.pose.position.x = -0.25
    pose.pose.position.y = -0.3
    pose.pose.position.z = 0.5 
    pose.pose.orientation.x = -0.72
    pose.pose.orientation.y = 0.7
    pose.pose.orientation.z = 0.0
    pose.pose.orientation.w = 0.012
    return pose

def teleoperation_publisher():
    rospy.init_node('teleoperation_publisher', anonymous=True)
    pub = rospy.Publisher('/desired_pose', PoseStamped, queue_size=10)
    rate = rospy.Rate(500)  # Publish at 50 Hz

    while not rospy.is_shutdown():
        pose = get_teleoperation_pose()  # Get the desired pose from your device
        pub.publish(pose)  # Publish it to /desired_pose
        rate.sleep()

if __name__ == '__main__':
    try:
        teleoperation_publisher()
    except rospy.ROSInterruptException:
        pass
