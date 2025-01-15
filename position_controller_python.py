#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import numpy as np
import tf.transformations as tf
from scipy.spatial.transform import Rotation as R

# Import robot kinematic libraries
from kdl_parser_py.urdf import treeFromUrdfModel
from urdf_parser_py.urdf import URDF
import PyKDL as kdl

def compute_orientation_error(tcp_orientation, desired_orientation):
    """
    Compute orientation error based on rotation matrices.
    tcp_orientation = Rtcp ==> 3x3 rotation matrix
    desired_orientation = Rdes ==> 3x3 rotation matrix
    DeltaR = Rdes * Rtcp.T
    orientation_error = Skew-symmetric part of DeltaR:
        e_x = DeltaR(2, 1) - DeltaR(1, 2)
        e_y = DeltaR(0, 2) - DeltaR(2, 0)
        e_z = DeltaR(1, 0) - DeltaR(0, 1)
    """
    delta_R = np.dot(desired_orientation, tcp_orientation.T)
    orientation_error = np.array([
        delta_R[2, 1] - delta_R[1, 2],
        delta_R[0, 2] - delta_R[2, 0],
        delta_R[1, 0] - delta_R[0, 1]
    ])
    return orientation_error

class LinearTimeLaw:
    ''' 
    Linear interpolation between two points in a given time duration.
    This outputs the path function p(s(t)), with s(t) being the time law.
    The formula is the following:
        p(s(t)) = x_i + (x_f - x_i) * s(t)
        - x_i: initial point
        - x_f: final point
        - t: specific time
        - dt: time span given by the publishing node (1 / 50)
        - t/dt = s(t) = alpha (time parametrization, s belongs to [0, 1])   
    '''
    def __init__(self, start, end, duration):
        self.start = start
        self.end = end
        self.duration = duration

    def evaluate(self, t):
        if t > self.duration:
            return self.end
        
        # This is the linear interpolation formula on the abscissa: t will take values multiples of 1/500=0.002
        alpha = t / self.duration

        # What you are returning now is p(s(t)) = x_i + (x_f - x_i) * s(t)
        return self.start + alpha * (self.end - self.start)

class Interpolator: 
    ''' 
    Interpolator class to generate a trajectory between two points in a given time duration.
    The path function is a function of time s(t) that returns 
        the desired point at time t (you can define a lambda function that works with 'LinearTimeLaw').
    '''
    def __init__(self, path_function, duration):
        self.path_function = path_function
        self.duration = duration

    def discretize_trajectory(self, rate, start_time):
        sampling_time = 1.0 / rate # 0.002 in case of 500 Hz
        time = start_time # Usually 0 at the beginning
        points = []

        # Until you reach the time taken by the publishing node to send the desired pose
        while time < self.duration:
            points.append(self.path_function(time))
            time += sampling_time

        remaining_time = max(0.0, self.duration - time)
        return points, remaining_time

class Kinematics:
    def __init__(self, robot_description):
        success, tree = treeFromUrdfModel(URDF.from_xml_string(robot_description))
        if not success:
            rospy.logerr("Failed to construct KDL tree")
            return
        self.tree = tree

    def set_chain(self, start, end):
        self.chain = self.tree.getChain(start, end)
        if not self.chain:
            rospy.logerr(f"Failed to create KDL chain from {start} to {end}.")
            return
        rospy.loginfo(f"KDL chain created with {self.chain.getNrOfJoints()} joints.")
        self.fk_solver = kdl.ChainFkSolverPos_recursive(self.chain)
        self.jac_solver = kdl.ChainJntToJacSolver(self.chain)

    def compute_fk(self, joints):
        joint_array = kdl.JntArray(len(joints))
        for i, joint in enumerate(joints):
            joint_array[i] = joint
        cartesian_pose = kdl.Frame()
        result = self.fk_solver.JntToCart(joint_array, cartesian_pose)
        if result < 0:
            rospy.logerr("Failed to calculate forward kinematics")
            return None
        return cartesian_pose

    def compute_jacobian(self, joints):
        joint_array = kdl.JntArray(len(joints))
        for i, joint in enumerate(joints):
            joint_array[i] = joint

        jacobian = kdl.Jacobian(len(joints))
        if self.jac_solver.JntToJac(joint_array, jacobian) < 0:
            rospy.logerr("Failed to calculate Jacobian")
            return None

        jacobian_np = np.zeros((jacobian.rows(), jacobian.columns()))
        for i in range(jacobian.rows()):
            for j in range(jacobian.columns()):
                jacobian_np[i, j] = jacobian[i, j]
        return jacobian_np

class Controller:
    def __init__(self, nh, control_rate):
        self.nh = nh
        self.control_rate = control_rate
        self.kp = np.array([1, 1, 1])
        self.ko = np.array([1, 1, 1])
        self.joint_state_sub = rospy.Subscriber("/joint_states", JointState, self.joint_state_callback)
        self.desired_pose_sub = rospy.Subscriber("/desired_pose", PoseStamped, self.desired_pose_callback)
        self.joint_velocity_pub = rospy.Publisher("/joint_group_vel_controller/command", Float64MultiArray, queue_size=1)
        self.joint_positions = np.zeros(6)
        self.desired_pose = None
        self.interpolator_position = None
        self.interpolator_orientation = None
        self.buffered_position = []
        self.buffered_orientation = []
        robot_description = rospy.get_param("robot_description", "")
        self.kinematics = Kinematics(robot_description)
        self.kinematics.set_chain("teleop_link", "tool0")

    def joint_state_callback(self, msg):
        joint_names = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
        joint_index_map = {name: i for i, name in enumerate(msg.name)}
        self.joint_positions = np.zeros(len(joint_names))
        for i, joint_name in enumerate(joint_names):
            if joint_name in joint_index_map:
                self.joint_positions[i] = msg.position[joint_index_map[joint_name]]

    def desired_pose_callback(self, msg):
        if self.desired_pose is None:
            self.desired_pose = msg.pose
            self.prev_position = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
            q = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
            self.prev_orientation = R.from_quat(q).as_matrix()
        else:
            delta_time = 1.0 / 100.0 # put here teh frequency of the node that publsihes the desired pose
            current_position = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
            q = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
            current_orientation = R.from_quat(q).as_matrix()
            self.create_interpolators(self.prev_position, current_position, self.prev_orientation, current_orientation, delta_time)
            self.prev_position = current_position
            self.prev_orientation = current_orientation


    def create_interpolators(self, start_position, end_position, start_orientation, end_orientation, duration):
        self.interpolator_position = Interpolator(
            lambda t: LinearTimeLaw(start_position, end_position, duration).evaluate(t), 
            duration
        )

        ''' 
        Compute the axis angle representation of the rotation matrix.
        start_orientation = Rstart ==> 3x3 rotation matrix
        end_orientation = Rend ==> 3x3 rotation matrix
        R^i_f = deltaR = Rstart^T * Rend
        NOTE: the code handles the case where the rotation is zero, that could lead to numerical
            instability when computing the norm of the axis_angle vector (you should divide by zero).
        '''
        delta_R = np.dot(start_orientation.T, end_orientation)

        # Pass to axis-angle representation (delta_R => r_vector = axis_angle)
        # np.linalg.norm = ||axis_angle|| = rotation angle
        axis_angle = R.from_matrix(delta_R).as_rotvec()
      
        # Handle zero rotation case
        if np.linalg.norm(axis_angle) < 1e-1:  # Threshold to check for near-zero rotation
            self.interpolator_orientation = Interpolator(
                lambda t: start_orientation,  # No rotation; keep the start orientation
                duration
            )
        else:
            # LinearTimeLaw(0, np.linalg.norm(axis_angle), duration).evaluate => s(t_k)
            # LinearTimeLaw(0, np.linalg.norm(axis_angle), duration).evaluate(t) * axis_angle = s(t_k) * r_vector
            # R.from_rotvec(s(t_k) * r_vector) = matrix exponential exp(s(t_k) * r_vector)
            # R_prev * exp(s(t_k) * r_vector) = R_desired (implicit in the interpolator)
            self.interpolator_orientation = Interpolator(
                lambda t: R.from_rotvec(
                    LinearTimeLaw(0, np.linalg.norm(axis_angle), duration).evaluate(t) * 
                    axis_angle / np.linalg.norm(axis_angle)
                ).as_matrix(),
                duration
            )

        self.buffered_position, _ = self.interpolator_position.discretize_trajectory(self.control_rate, 0.0)
        self.buffered_orientation, _ = self.interpolator_orientation.discretize_trajectory(self.control_rate, 0.0)


    def compute_control_action(self):
        if not self.buffered_position or not self.buffered_orientation:
            return

        # Compute the 'feedback' cartesian value
        tcp_pose = self.kinematics.compute_fk(self.joint_positions)
        if tcp_pose is None:
            return

        tcp_position = np.array([tcp_pose.p[0], tcp_pose.p[1], tcp_pose.p[2]])
        tcp_orientation = np.array([
            [tcp_pose.M[0, 0], tcp_pose.M[0, 1], tcp_pose.M[0, 2]],
            [tcp_pose.M[1, 0], tcp_pose.M[1, 1], tcp_pose.M[1, 2]],
            [tcp_pose.M[2, 0], tcp_pose.M[2, 1], tcp_pose.M[2, 2]]
        ])

        # Thanks to the interpolator, we can now get the desired position and orientation
        desired_position = self.buffered_position.pop(0)
        desired_orientation = self.buffered_orientation.pop(0)

        # Calculate the errors (watch out for the orientation error...)
        position_error = desired_position - tcp_position
        orientation_error = compute_orientation_error(tcp_orientation, desired_orientation)

        # Calculate the control input
        control_input_p = self.kp * position_error
        control_input_o = self.ko * orientation_error
        control_input = np.hstack((control_input_p, control_input_o))

        # Pass from cartesian to joint space
        jacobian = self.kinematics.compute_jacobian(self.joint_positions)
        if jacobian is None or np.linalg.det(jacobian) == 0:
            rospy.logerr("Jacobian is singular or invalid.")
            return
        joint_velocities = np.dot(np.linalg.pinv(jacobian), control_input)
        self.publish_joint_velocities(joint_velocities) # publish at 500 Hz

    def publish_joint_velocities(self, joint_velocities):
        msg = Float64MultiArray()
        msg.data = joint_velocities.tolist()
        self.joint_velocity_pub.publish(msg)

if __name__ == "__main__":
    rospy.init_node("position_controller")
    control_rate = 500.0
    rate = rospy.Rate(control_rate)
    controller = Controller(rospy, control_rate)

    while not rospy.is_shutdown():
        controller.compute_control_action()
        rate.sleep()