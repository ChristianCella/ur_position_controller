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
import numpy as np

def compute_orientation_error(tcp_orientation, desired_orientation):
    """
    Compute orientation error based on rotation matrices.
    """
    # Compute delta_R
    delta_R = np.dot(desired_orientation, tcp_orientation.T)

    delta_R_4x4 = np.eye(4)
    delta_R_4x4[:3, :3] = delta_R

    # Convert delta_R to quaternion
    ##delta_q = tf.quaternion_from_matrix(np.vstack((np.hstack((delta_R, [[0], [0], [0]])), [0, 0, 0, 1])))
    delta_q = tf.quaternion_from_matrix(delta_R_4x4)
    delta_q_negated = [-delta_q[0], -delta_q[1], -delta_q[2], -delta_q[3]]

    # Compute shortest rotation
    angle_delta_q = 2 * np.arccos(delta_q[3])
    angle_delta_q_negated = 2 * np.arccos(delta_q_negated[0])

    if abs(angle_delta_q) > abs(angle_delta_q_negated):
        delta_q = delta_q_negated

    # Return vector part of the quaternion as the orientation error
    return np.array([delta_q[0], delta_q[1], delta_q[2]])

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
        rospy.loginfo(f"KDL chain joint names: {self.get_chain_joint_names()}")
        self.fk_solver = kdl.ChainFkSolverPos_recursive(self.chain)
        self.jac_solver = kdl.ChainJntToJacSolver(self.chain)



    def get_chain_joint_names(self):
        """
        Extract the joint names from the KDL chain while filtering for movable joints.
        """
        joint_names = []
        for i in range(self.chain.getNrOfSegments()):
            segment = self.chain.getSegment(i)
            joint = segment.getJoint()
            # Include only movable joints (Rotational and Prismatic)
            if joint.getType() in [kdl.Joint.RotAxis, kdl.Joint.TransAxis]:
                joint_names.append(joint.getName())
            else:
                rospy.logwarn(f"Excluding joint: {joint.getName()} (type: {joint.getType()})")
        rospy.loginfo(f"Identified movable KDL chain joint names: {joint_names}")
        return joint_names

    def compute_fk(self, joints):
        if len(joints) != self.chain.getNrOfJoints():
            rospy.logerr(f"Mismatch between number of joints ({len(joints)}) and KDL chain ({self.chain.getNrOfJoints()}).")
            return None

        joint_array = kdl.JntArray(len(joints))
        for i, joint in enumerate(joints):
            joint_array[i] = joint

        cartesian_pose = kdl.Frame()
        result = self.fk_solver.JntToCart(joint_array, cartesian_pose)
        if result < 0:
            rospy.logerr(f"Failed to calculate forward kinematics. Error code: {result}")
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
        # Convert KDL Jacobian to a NumPy array
        jacobian_np = np.zeros((jacobian.rows(), jacobian.columns()))
        for i in range(jacobian.rows()):
            for j in range(jacobian.columns()):
                jacobian_np[i, j] = jacobian[i, j]
        return jacobian_np

class Controller:
    def __init__(self, nh, control_rate):
        self.nh = nh
        self.control_rate = control_rate
        self.kp = np.array([0.5, 0.5, 0.5])
        self.ko = np.array([0.5, 0.5, 0.5])
        self.joint_velocities = np.zeros(6)

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

        # Store the joint names from the message
        self.last_joint_state_names = msg.name

        # Map joint names to indices in the incoming message
        joint_index_map = {name: i for i, name in enumerate(msg.name)}

        # Initialize joint positions array
        self.joint_positions = np.zeros(len(joint_names))

        # Populate joint positions in the order of `joint_names`
        for i, joint_name in enumerate(joint_names):
            if joint_name in joint_index_map:
                self.joint_positions[i] = msg.position[joint_index_map[joint_name]]
            else:
                rospy.logwarn(f"Joint {joint_name} not found in JointState message.")

        # Log filtered positions and names
        rospy.loginfo(f"Filtered joint positions: {self.joint_positions}")
        rospy.loginfo(f"Filtered joint names: {joint_names}")



    def desired_pose_callback(self, msg):
        self.desired_pose = msg.pose

    def compute_control_action(self):
        if self.desired_pose is None:
            rospy.logwarn("No desired pose received yet.")
            return
        rospy.loginfo(f"THE DESIRED POSE IS: {self.desired_pose}")  

        fk_pose = self.kinematics.compute_fk(self.joint_positions)
        if fk_pose is None:
            return

        tcp_position = np.array([fk_pose.p[0], fk_pose.p[1], fk_pose.p[2]])
        desired_position = np.array([self.desired_pose.position.x, self.desired_pose.position.y, self.desired_pose.position.z])
        
        rospy.loginfo(f"TCP position: {tcp_position}")
        rospy.loginfo(f"Desired position: {desired_position}")
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

        rospy.loginfo(f"TCP orientation: {tcp_orientation}")
        rospy.loginfo(f"the desired quaternion is: {self.desired_pose.orientation}")

        orientation_error = compute_orientation_error(tcp_orientation, desired_orientation)
        rospy.loginfo(f"Orientation error: {orientation_error}")

        control_input_p = self.kp * position_error
        control_input_o = self.ko * orientation_error

        control_input = np.hstack((control_input_p, control_input_o))
        jacobian = self.kinematics.compute_jacobian(self.joint_positions)

        if jacobian is None or np.linalg.det(jacobian) == 0:
            rospy.logerr("Jacobian is singular or invalid.")
            return

        self.joint_velocities = np.dot(np.linalg.pinv(jacobian), control_input)
        rospy.loginfo(f"Joint velocities before adjustment: {self.joint_velocities}")
        self.publish_joint_velocities()

    def publish_joint_velocities(self):
        msg = Float64MultiArray()

        # Ensure joint state names are available
        if not hasattr(self, 'last_joint_state_names') or not self.last_joint_state_names:
            rospy.logerr("No joint state names available for reordering velocities. Skipping publish.")
            return

        # Define the robot joint names
        robot_joint_names = ["elbow_joint", "shoulder_lift_joint", "shoulder_pan_joint",
                            "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]

        # Reorder velocities for robot joints
        reordered_velocities = []
        for joint_name in robot_joint_names:
            if joint_name in self.last_joint_state_names:
                # Find the index of the joint in the last JointState message
                joint_index_in_state = self.last_joint_state_names.index(joint_name)

                # Append the corresponding velocity
                reordered_velocities.append(self.joint_velocities[robot_joint_names.index(joint_name)])
            else:
                rospy.logwarn(f"Joint {joint_name} not found in current JointState message.")
                reordered_velocities.append(0.0)  # Default to 0 velocity for missing joints

        # Assign reordered velocities to the message
        msg.data = reordered_velocities

        # Log the joint order and velocities for debugging
        rospy.loginfo(f"Publishing joint velocities in order: {robot_joint_names}")
        rospy.loginfo(f"Joint velocities: {reordered_velocities}")

        # Publish the reordered velocities
        self.joint_velocity_pub.publish(msg)




if __name__ == "__main__":
    rospy.init_node("position_controller")
    control_rate = 50.0  # Hz
    rate = rospy.Rate(control_rate)

    controller = Controller(rospy, control_rate)

    while not rospy.is_shutdown():
        controller.compute_control_action()
        rate.sleep()
