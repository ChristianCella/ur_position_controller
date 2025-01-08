#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
import numpy as np
from math import pi
from scipy.spatial.transform import Rotation as R
import tf
from std_srvs.srv import Trigger

class RobotPose:

    '''Class constructor'''
    def __init__(self, angles: list, axis: list, tcp_position: list, frame_id: str) -> None:
        self.angles = angles
        self.axis = axis
        self.tcp_position = tcp_position
        self.frame_id = frame_id

    '''Class methods'''
    def rotation_matrix_x(self, angle) -> np.ndarray:
        ''' 
        Compute the rotation matrix around the x axis
        - angle: angle in radians
        '''
        return np.array([
            [1, 0, 0],
            [0, np.cos(angle), -np.sin(angle)],
            [0, np.sin(angle), np.cos(angle)]
        ])
    
    def rotation_matrix_y(self, angle) -> np.ndarray:
        ''' 
        Compute the rotation matrix around the y axis
        - angle: angle in radians
        '''
        return np.array([
            [np.cos(angle), 0, np.sin(angle)],
            [0, 1, 0],
            [-np.sin(angle), 0, np.cos(angle)]
        ])
    
    def rotation_matrix_z(self, angle) -> np.ndarray:
        ''' 
        Compute the rotation matrix around the z axis
        - angle: angle in radians
        '''
        return np.array([
            [np.cos(angle), -np.sin(angle), 0],
            [np.sin(angle), np.cos(angle), 0],
            [0, 0, 1]
        ])  

    def matrix_to_quaternion(self, Rot) -> list:
        '''
        Convert a rotation matrix to a quaternion
        - Rot: 3x3 rotation matrix (specified as a NumPy array)
        '''
        quat = (R.from_matrix(Rot)).as_quat()
        return quat

    def compute_pose(self) -> PoseStamped:
        ''' 
        Compute the pose of the robot end-effector
        '''
        self.rot = []
        for i in self.axis:
            if i == 'x':
                self.rotx = self.rotation_matrix_x(self.angles[self.axis.index(i)])
                self.rot.append(self.rotx)
            elif i == 'y':
                self.roty = self.rotation_matrix_y(self.angles[self.axis.index(i)])
                self.rot.append(self.roty)
            elif i == 'z':
                self.rotz = self.rotation_matrix_z(self.angles[self.axis.index(i)])
                self.rot.append(self.rotz)

        # Compute the overall rotation matrix and then convert it to a quaternion
        if len(self.rot) == 0: # No rotation
            self.rot_mat = np.eye(3)
        elif len(self.rot) == 1: # Single rotation
            self.rot_mat = self.rot[0]
        else: # More than one rotation
            self.rot_mat = np.linalg.multi_dot(self.rot)

        # Get the quaternion
        self.quat = self.matrix_to_quaternion(self.rot_mat)

        # Define the pose
        self.pose = PoseStamped()
        self.pose.header.frame_id = self.frame_id
        self.pose.pose.position.x = self.tcp_position[0]
        self.pose.pose.position.y = self.tcp_position[1]
        self.pose.pose.position.z = self.tcp_position[2]
        self.pose.pose.orientation.x = self.quat[0]
        self.pose.pose.orientation.y = self.quat[1]
        self.pose.pose.orientation.z = self.quat[2]
        self.pose.pose.orientation.w = self.quat[3]

        return self.pose
       

class PosePublisher:

    '''Class constructor'''
    def __init__(self, topic_name: str, pub_freq: float, node_name: str, tcp_pose: list, error: float) -> None:
        self.topic_name = topic_name
        self.pub_freq = pub_freq
        self.node_name = node_name # 'teleoperation_publisher'
        self.tcp_pose = tcp_pose
        self.error = error
        self.verbose = True

        #self.cl_openGripper = rospy.ServiceProxy('open_gripper', Trigger)
        #self.cl_closeGripper = rospy.ServiceProxy('close_gripper', Trigger)
        
        # Initialize the node 
        rospy.init_node(self.node_name, anonymous=True)
        if self.verbose: rospy.loginfo('Initialized teleoperation_publisher node')

        # Initialize the publisher object
        self.pub = rospy.Publisher(self.topic_name, PoseStamped, queue_size=10)

        # Initialize the transform listener that subscribes to the tf topic
        self.listener = tf.TransformListener()

        # Wait for TF to have data
        rospy.sleep(1.0)

    def teleoperation_publisher(self) -> None:
        self.rate = rospy.Rate(self.pub_freq)
        i = 0
        while not rospy.is_shutdown():

            # Publish the desired pose
            if i >= len(self.tcp_pose): # Keep providing the final pose
                self.current_pose = self.tcp_pose[-1]
            else:
                self.current_pose = self.tcp_pose[i]
            self.pub.publish(self.current_pose)
            #self.cl_closeGripper()
            #rospy.sleep(0.25)
            #rospy.loginfo(f"Closed gripper")

            if self.verbose: rospy.loginfo(f"Publishing desired pose {i+1}")
            self.verbose = False

            try:
                (translation, rotation) = self.listener.lookupTransform(
                    self.current_pose.header.frame_id, 'tool0', rospy.Time(0)
                )
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                rospy.logwarn("Could not lookup transform: %s", e)
                self.rate.sleep()
                continue

            # Compare actual (translation) vs. desired (current_pose)
            x, y, z = translation
            if (abs(x - self.current_pose.pose.position.x) < self.error and
                abs(y - self.current_pose.pose.position.y) < self.error and
                abs(z - self.current_pose.pose.position.z) < self.error):

                rospy.loginfo(f"Reached desired pose {i+1}")
                i += 1
                self.verbose = True
                #self.cl_openGripper()
                #rospy.sleep(0.25)
                #rospy.loginfo(f"Opened gripper")

            self.rate.sleep()

# Test the class
if __name__ == '__main__':
    
    # Define all the poses
    robot_pose1: RobotPose = RobotPose(angles = [pi, -pi/2], axis = ['x', 'z'], tcp_position = [-0.30, 0.50, 0.30], frame_id = 'base_link')
    robot_pose2: RobotPose = RobotPose(angles = [pi], axis = ['x'], tcp_position = [0.30, 0.35, 0.40], frame_id='base_link')
    robot_pose3: RobotPose = RobotPose(angles = [pi], axis = ['x'], tcp_position = [-0.30, 0.5, 0.3], frame_id='base_link')
    poses = [robot_pose1.compute_pose(), robot_pose2.compute_pose(), robot_pose3.compute_pose()]

    # Instantiate the publisher
    pose_publisher: PosePublisher = PosePublisher(topic_name='/desired_pose', 
                                                  pub_freq=50.0, 
                                                  node_name='teleoperation_publisher', 
                                                  tcp_pose=poses, 
                                                  error=0.01)

    # Main code
    try:
        pose_publisher.teleoperation_publisher()
    except rospy.ROSInterruptException:
        pass

