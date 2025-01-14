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
    delta_R = np.dot(desired_orientation, tcp_orientation.T)
    orientation_error = np.array([
        delta_R[2, 1] - delta_R[1, 2],
        delta_R[0, 2] - delta_R[2, 0],
        delta_R[1, 0] - delta_R[0, 1]
    ])
    return orientation_error

class LinearTimeLaw:
    def __init__(self, start, end, duration):
        self.start = start
        self.end = end
        self.duration = duration

    def evaluate(self, t):
        if t > self.duration:
            return self.end
        alpha = t / self.duration
        return self.start + alpha * (self.end - self.start)

class Interpolator:
    def __init__(self, path_function, duration):
        self.path_function = path_function
        self.duration = duration

    def discretize_trajectory(self, rate, start_time):
        sampling_time = 1.0 / rate
        time = start_time
        points = []

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
        self.kinematics.set_chain("base_link", "tool0")

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
            delta_time = 1.0 / 50.0
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

        delta_R = np.dot(end_orientation, start_orientation.T)
        axis_angle = R.from_matrix(delta_R).as_rotvec()

        # Handle zero rotation case
        if np.linalg.norm(axis_angle) < 1e-6:  # Threshold to check for near-zero rotation
            self.interpolator_orientation = Interpolator(
                lambda t: start_orientation,  # No rotation; keep the start orientation
                duration
            )
        else:
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

        tcp_pose = self.kinematics.compute_fk(self.joint_positions)
        if tcp_pose is None:
            return

        tcp_position = np.array([tcp_pose.p[0], tcp_pose.p[1], tcp_pose.p[2]])
        tcp_orientation = np.array([
            [tcp_pose.M[0, 0], tcp_pose.M[0, 1], tcp_pose.M[0, 2]],
            [tcp_pose.M[1, 0], tcp_pose.M[1, 1], tcp_pose.M[1, 2]],
            [tcp_pose.M[2, 0], tcp_pose.M[2, 1], tcp_pose.M[2, 2]]
        ])

        desired_position = self.buffered_position.pop(0)
        desired_orientation = self.buffered_orientation.pop(0)

        position_error = desired_position - tcp_position
        orientation_error = compute_orientation_error(tcp_orientation, desired_orientation)

        control_input_p = self.kp * position_error
        control_input_o = self.ko * orientation_error
        control_input = np.hstack((control_input_p, control_input_o))

        jacobian = self.kinematics.compute_jacobian(self.joint_positions)
        if jacobian is None or np.linalg.det(jacobian) == 0:
            rospy.logerr("Jacobian is singular or invalid.")
            return

        joint_velocities = np.dot(np.linalg.pinv(jacobian), control_input)
        self.publish_joint_velocities(joint_velocities)

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
