#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped

""" 
        x: 0.0
        y: 0.0996
        z: -2.042830148012698e-11
      rotation: 
        x: -0.7071067799551687
        y: -4.294163923668199e-05
        z: -4.29416392279611e-05
        w: 0.7071067798101386

"""

# Replace this with code to get the pose from your device
def get_teleoperation_pose():
    # Example pose, replace with your teleoperation device's logic
    pose = PoseStamped()
    pose.header.stamp = rospy.Time.now()
    pose.header.frame_id = "base"  # Change to the appropriate frame
    pose.pose.position.x = 0.30
    pose.pose.position.y = 0.40
    pose.pose.position.z = 0.40
    pose.pose.orientation.x = 0.998
    pose.pose.orientation.y = 0.001
    pose.pose.orientation.z = -0.022
    pose.pose.orientation.w = 0.065
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
