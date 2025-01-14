#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import numpy as np
from kdl_parser_py.urdf import treeFromUrdfModel
from urdf_parser_py.urdf import URDF
import PyKDL as kdl
import tf.transformations as tf

def compute_orientation_error(tcp_orientation, desired_orientation):
    """
    Compute orientation error based on rotation matrices.
    """
    delta_R = np.dot(desired_orientation, tcp_orientation.T)
    orientation_error = np.array([
        delta_R[2, 1] - delta_R[1, 2],
        delta_R[0, 2] - delta_R[2, 0],
        delta_R[1, 0] - delta_R[0, 1]
    ])
    return orientation_error

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

        robot_description = rospy.get_param("robot_description", "")
        self.kinematics = Kinematics(robot_description)
        self.kinematics.set_chain("base_link", "tool0")

    def joint_state_callback(self, msg):
        joint_names = ["shoulder_pan_joint", "shoulder_lift_joint", 
                       "elbow_joint", "wrist_1_joint", 
                       "wrist_2_joint", "wrist_3_joint"]

        joint_index_map = {name: i for i, name in enumerate(msg.name)}

        self.joint_positions = np.zeros(len(joint_names))
        for i, joint_name in enumerate(joint_names):
            if joint_name in joint_index_map:
                self.joint_positions[i] = msg.position[joint_index_map[joint_name]]

    def desired_pose_callback(self, msg):
        self.desired_pose = msg.pose

    def compute_control_action(self):
        if self.desired_pose is None:
            return

        fk_pose = self.kinematics.compute_fk(self.joint_positions)
        if fk_pose is None:
            return

        tcp_position = np.array([fk_pose.p[0], fk_pose.p[1], fk_pose.p[2]])
        desired_position = np.array([self.desired_pose.position.x, self.desired_pose.position.y, self.desired_pose.position.z])
        position_error = desired_position - tcp_position

        tcp_orientation = np.array([
            [fk_pose.M[0, 0], fk_pose.M[0, 1], fk_pose.M[0, 2]],
            [fk_pose.M[1, 0], fk_pose.M[1, 1], fk_pose.M[1, 2]],
            [fk_pose.M[2, 0], fk_pose.M[2, 1], fk_pose.M[2, 2]]
        ])

        desired_orientation = tf.quaternion_matrix([
            self.desired_pose.orientation.x,
            self.desired_pose.orientation.y,
            self.desired_pose.orientation.z,
            self.desired_pose.orientation.w
        ])[:3, :3]

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

        rospy.loginfo("Control action computed.")

    def publish_joint_velocities(self, joint_velocities):
        msg = Float64MultiArray()
        msg.data = joint_velocities.tolist()
        self.joint_velocity_pub.publish(msg)

if __name__ == "__main__":
    rospy.init_node("position_controller")
    control_rate = 50.0
    rate = rospy.Rate(control_rate)

    controller = Controller(rospy, control_rate)

    while not rospy.is_shutdown():
        controller.compute_control_action()
        rate.sleep()
