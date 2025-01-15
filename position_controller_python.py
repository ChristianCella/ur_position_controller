#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import numpy as np
from scipy.spatial.transform import Rotation as R
from ros_dmp.msg import CartesianTrajectory
from kdl_parser_py.urdf import treeFromUrdfModel
from urdf_parser_py.urdf import URDF
import PyKDL as kdl

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
        self.ik_solver = kdl.ChainIkSolverPos_LMA(self.chain)
        self.jac_solver = kdl.ChainJntToJacSolver(self.chain)
        self.fk_solver = kdl.ChainFkSolverPos_recursive(self.chain)

    def compute_fk(self, joints):
        joint_array = kdl.JntArray(len(joints))
        for i, joint in enumerate(joints):
            joint_array[i] = joint

        cartesian_pose = kdl.Frame()
        if self.fk_solver.JntToCart(joint_array, cartesian_pose) < 0:
            rospy.logerr("Failed to compute forward kinematics")
            return None
        return cartesian_pose

    def compute_ik(self, desired_pose, current_joint_positions):
        joint_array = kdl.JntArray(len(current_joint_positions))
        for i, joint in enumerate(current_joint_positions):
            joint_array[i] = joint

        try:
            rotation_matrix = kdl.Rotation(
                desired_pose[0, 0], desired_pose[0, 1], desired_pose[0, 2],
                desired_pose[1, 0], desired_pose[1, 1], desired_pose[1, 2],
                desired_pose[2, 0], desired_pose[2, 1], desired_pose[2, 2]
            )
        except Exception as e:
            rospy.logerr(f"Invalid rotation matrix: {e}")
            return None

        position_vector = kdl.Vector(
            desired_pose[0, 3], desired_pose[1, 3], desired_pose[2, 3]
        )
        kdl_pose = kdl.Frame(rotation_matrix, position_vector)

        result = kdl.JntArray(len(current_joint_positions))
        if self.ik_solver.CartToJnt(joint_array, kdl_pose, result) < 0:
            rospy.logerr("Failed to calculate inverse kinematics")
            return None

        return np.array([result[i] for i in range(result.rows())])

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

class Interpolator:
    def __init__(self, trajectory_points, duration):
        self.trajectory_points = trajectory_points
        self.duration = duration

    def interpolate(self, t):
        num_points = len(self.trajectory_points)
        if num_points == 1:
            return self.trajectory_points[0]

        if t >= self.duration:
            return self.trajectory_points[-1]

        idx = int((t / self.duration) * (num_points - 1))
        alpha = (t / self.duration) * (num_points - 1) - idx

        return self.trajectory_points[idx] * (1 - alpha) + self.trajectory_points[idx + 1] * alpha

class Controller:
    def __init__(self, nh, control_rate):
        self.nh = nh
        self.control_rate = control_rate
        self.joint_state_sub = rospy.Subscriber("/joint_states", JointState, self.joint_state_callback)
        self.trajectory_sub = rospy.Subscriber("/generate_motion_service_node/cartesian_trajectory", CartesianTrajectory, self.trajectory_callback)
        self.joint_velocity_pub = rospy.Publisher("/joint_group_vel_controller/command", Float64MultiArray, queue_size=1)
        self.joint_positions = np.zeros(6)
        self.interpolator_position = None
        self.interpolator_orientation = None
        self.interpolator_velocity = None
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

    def trajectory_callback(self, msg):
        positions = []
        orientations = []
        velocities = []

        rospy.loginfo(f"The current published pose is {msg.cartesian_state[0].pose}")

        for state in msg.cartesian_state:
            positions.append([state.pose.position.x, state.pose.position.y, state.pose.position.z])
            orientations.append([
                state.pose.orientation.x,
                state.pose.orientation.y,
                state.pose.orientation.z,
                state.pose.orientation.w
            ])
            velocities.append([
                state.vel.linear.x, state.vel.linear.y, state.vel.linear.z,
                state.vel.angular.x, state.vel.angular.y, state.vel.angular.z
            ])

        duration = len(msg.cartesian_state) / self.control_rate
        self.interpolator_position = Interpolator(np.array(positions), duration)
        self.interpolator_orientation = Interpolator(np.array(orientations), duration)
        self.interpolator_velocity = Interpolator(np.array(velocities), duration)


    def compute_and_publish_joint_velocities(self):
        if not self.interpolator_position or not self.interpolator_orientation or not self.interpolator_velocity:
            return

        t = rospy.get_time() % self.interpolator_position.duration

        # Interpolated position, orientation, and velocity
        desired_position_delta = self.interpolator_position.interpolate(t)
        desired_orientation_quat = self.interpolator_orientation.interpolate(t)
        desired_orientation = R.from_quat(desired_orientation_quat).as_matrix()
        cartesian_velocity = self.interpolator_velocity.interpolate(t)

        # Get current TCP position from forward kinematics
        current_tcp_pose = self.kinematics.compute_fk(self.joint_positions)
        if current_tcp_pose is None:
            return

        # Extract current position and compute the new desired position
        current_position = np.array([current_tcp_pose.p[0], current_tcp_pose.p[1], current_tcp_pose.p[2]])
        desired_position = current_position + desired_position_delta

        # Build the desired pose matrix
        desired_tcp_pose = np.eye(4)
        desired_tcp_pose[:3, 3] = desired_position
        desired_tcp_pose[:3, :3] = desired_orientation

        #rospy.loginfo(f"Desired position: {desired_tcp_pose}")

        # Compute joint positions via inverse kinematics
        joint_positions = self.kinematics.compute_ik(desired_tcp_pose, self.joint_positions)
        if joint_positions is None:
            return

        # Compute Jacobian for these joint positions
        jacobian = self.kinematics.compute_jacobian(joint_positions)
        if jacobian is None or jacobian.shape[0] != 6:
            rospy.logerr("Jacobian is singular or invalid.")
            return

        # Compute joint velocities from Cartesian velocity
        joint_velocities = np.dot(np.linalg.pinv(jacobian), cartesian_velocity)
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
        controller.compute_and_publish_joint_velocities()
        rate.sleep()
