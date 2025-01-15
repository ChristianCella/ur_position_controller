#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import numpy as np
import time
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

class Kinematics:

    ''' Class constructor '''
    def __init__(self, robot_description):
        success, tree = treeFromUrdfModel(URDF.from_xml_string(robot_description))
        if not success:
            rospy.logerr("Failed to construct KDL tree")
            return
        
        # Initialize the KDL kinematics tree
        self.tree = tree

    def set_chain(self, start, end):
        ''' 
        Define the kinematic chain from the start link to the end link.
        '''
        self.chain = self.tree.getChain(start, end)
        if not self.chain:
            rospy.logerr(f"Failed to create KDL chain from {start} to {end}.")
            return
        rospy.loginfo(f"KDL chain created with {self.chain.getNrOfJoints()} joints.")
        self.fk_solver = kdl.ChainFkSolverPos_recursive(self.chain)
        self.jac_solver = kdl.ChainJntToJacSolver(self.chain)

    def compute_fk(self, joints):
        ''' 
        Compute the forward kinematics given the joints positions.
        The joints positions are updated with the joint_state_callback method.
        '''
        joint_array = kdl.JntArray(len(joints))
        for i, joint in enumerate(joints):
            joint_array[i] = joint

        # Compute the cartesian psoe starting from the joints positions
        cartesian_pose = kdl.Frame()
        result = self.fk_solver.JntToCart(joint_array, cartesian_pose)
        if result < 0:
            rospy.logerr("Failed to calculate forward kinematics")
            return None

        return cartesian_pose

    def compute_jacobian(self, joints):
        ''' 
        Compute the Jacobian matrix given the joints positions.
        - x_dot = J(q) * q_dot ==> q_dot = J^+ * x_dot
        - x_dot = [v_(1x3), w_(1x3)]^T
        In this mthod, we only compute:
        - J(q) = [J_v(q), J_w(q)], with 
            - J_(v,i) = z_(i-1) x (p_n - p_(i-1))
            - J_(w,i) = z_(i-1)
        '''
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

class TimeLaw:
    """
    Base class for all time laws. It defines the interface for evaluate,
    evaluate_derivative, and get_parameters methods.
    """
    def evaluate(self, t):
        raise NotImplementedError("Evaluate method must be implemented in subclasses.")

    def evaluate_derivative(self, t):
        raise NotImplementedError("Evaluate derivative method must be implemented in subclasses.")

    def get_parameters(self):
        raise NotImplementedError("Get parameters method must be implemented in subclasses.")

class LinearTimeLaw(TimeLaw):
    """
    Implements linear interpolation between two points.
    """
    def __init__(self, start, end, duration):
        self.start = start
        self.end = end
        self.duration = duration

    def evaluate(self, t):
        if t > self.duration:
            return self.end
        alpha = t / self.duration
        return self.start + alpha * (self.end - self.start)

    def evaluate_derivative(self, t):
        return (self.end - self.start) / self.duration

    def get_parameters(self):
        return [self.start, self.end]

class CubicalTimeLaw(TimeLaw):
    """
    Implements cubic interpolation between two points with specified velocities.
    """
    def __init__(self, start, end, start_velocity, end_velocity, duration):
        self.a0 = start
        self.a1 = start_velocity
        self.a2 = (-3 * (start - end) - (2 * start_velocity + end_velocity) * duration) / duration**2
        self.a3 = (2 * (start - end) + (start_velocity + end_velocity) * duration) / duration**3
        self.duration = duration

    def evaluate(self, t):
        if t > self.duration:
            return self.a0 + self.a1 * self.duration + self.a2 * self.duration**2 + self.a3 * self.duration**3
        return self.a0 + self.a1 * t + self.a2 * t**2 + self.a3 * t**3

    def evaluate_derivative(self, t):
        return self.a1 + 2 * self.a2 * t + 3 * self.a3 * t**2

    def get_parameters(self):
        return [self.a0, self.a1, self.a2, self.a3]

class QuinticTimeLaw(TimeLaw):
    """
    Implements quintic interpolation between two points with specified velocities and accelerations.
    """
    def __init__(self, start, end, start_velocity, end_velocity, start_acceleration, end_acceleration, duration):
        self.a0 = start
        self.a1 = start_velocity
        self.a2 = 0.5 * start_acceleration
        self.a3 = (20 * (end - start) - (8 * end_velocity + 12 * start_velocity) * duration -
                   (3 * end_acceleration - start_acceleration) * duration**2) / (2 * duration**3)
        self.a4 = (30 * (start - end) + (14 * end_velocity + 16 * start_velocity) * duration +
                   (3 * end_acceleration - 2 * start_acceleration) * duration**2) / (2 * duration**4)
        self.a5 = (12 * (end - start) - 6 * (end_velocity + start_velocity) * duration -
                   (end_acceleration - start_acceleration) * duration**2) / (2 * duration**5)
        self.duration = duration

    def evaluate(self, t):
        if t > self.duration:
            return (self.a0 + self.a1 * self.duration + self.a2 * self.duration**2 +
                    self.a3 * self.duration**3 + self.a4 * self.duration**4 + self.a5 * self.duration**5)
        return self.a0 + self.a1 * t + self.a2 * t**2 + self.a3 * t**3 + self.a4 * t**4 + self.a5 * t**5

    def evaluate_derivative(self, t):
        return self.a1 + 2 * self.a2 * t + 3 * self.a3 * t**2 + 4 * self.a4 * t**3 + 5 * self.a5 * t**4

    def get_parameters(self):
        return [self.a0, self.a1, self.a2, self.a3, self.a4, self.a5]

class Interpolator: 
    """ 
    Interpolator class to generate a trajectory between two points in a given time duration.
    The path function is a function of time s(t) that returns 
        the desired point at time t.
    """
    def __init__(self, path_function, duration):
        self.path_function = path_function
        self.duration = duration

    def discretize_trajectory(self, rate, start_time):
        sampling_time = 1.0 / rate  # 0.002 in case of 500 Hz
        time = start_time  # Usually 0 at the beginning
        points = []

        # Until you reach the time taken by the publishing node to send the desired pose
        while time < self.duration:
            points.append(self.path_function(time))
            time += sampling_time

        remaining_time = max(0.0, self.duration - time)
        return points, remaining_time

class Controller:
    def __init__(self, nh, control_rate, time_law_type="linear"):
        self.nh = nh
        self.control_rate = control_rate
        self.time_law_type = time_law_type
        self.kp = np.array([0.1, 0.1, 0.1])
        self.ko = np.array([0.1, 0.1, 0.1])
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
        self.prev_velocity = np.zeros(3)
        self.prev_acceleration = np.zeros(3)

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
            self.prev_velocity = np.zeros(3)
            self.prev_acceleration = np.zeros(3)
        else:
            delta_time = 1.0 / 100.0
            current_position = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
            q = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
            current_orientation = R.from_quat(q).as_matrix()

            current_velocity = (current_position - self.prev_position) / delta_time
            current_acceleration = (current_velocity - self.prev_velocity) / delta_time

            self.create_interpolators(
                self.prev_position, current_position, 
                self.prev_orientation, current_orientation, 
                delta_time,
                start_velocity=self.prev_velocity, end_velocity=current_velocity,
                start_acceleration=self.prev_acceleration, end_acceleration=current_acceleration
            )

            self.prev_position = current_position
            self.prev_orientation = current_orientation
            self.prev_velocity = current_velocity
            self.prev_acceleration = current_acceleration

    def create_time_law(self, start, end, duration, start_velocity=None, end_velocity=None, start_acceleration=None, end_acceleration=None):
        """
        Factory method to create the desired time law based on the user's choice.
        Handles both scalar and array inputs for velocity and acceleration.
        """
        start_velocity = start_velocity if start_velocity is not None else np.zeros_like(start)
        end_velocity = end_velocity if end_velocity is not None else np.zeros_like(end)
        start_acceleration = start_acceleration if start_acceleration is not None else np.zeros_like(start)
        end_acceleration = end_acceleration if end_acceleration is not None else np.zeros_like(end)

        if self.time_law_type == "linear":
            return LinearTimeLaw(start, end, duration)
        elif self.time_law_type == "cubic":
            return CubicalTimeLaw(start, end, start_velocity, end_velocity, duration)
        elif self.time_law_type == "quintic":
            return QuinticTimeLaw(start, end, start_velocity, end_velocity, start_acceleration, end_acceleration, duration)
        else:
            raise ValueError(f"Unsupported time law type: {self.time_law_type}")


    def create_interpolators(self, start_position, end_position, start_orientation, end_orientation, duration, 
                             start_velocity=None, end_velocity=None, start_acceleration=None, end_acceleration=None):
        time_law_position = self.create_time_law(start_position, end_position, duration, 
                                                 start_velocity=start_velocity, end_velocity=end_velocity, 
                                                 start_acceleration=start_acceleration, end_acceleration=end_acceleration)
        self.interpolator_position = Interpolator(lambda t: time_law_position.evaluate(t), duration)

        delta_R = np.dot(start_orientation.T, end_orientation)
        axis_angle = R.from_matrix(delta_R).as_rotvec()

        if np.linalg.norm(axis_angle) < 1e-1:
            self.interpolator_orientation = Interpolator(lambda t: start_orientation, duration)
        else:
            time_law_orientation = self.create_time_law(0, np.linalg.norm(axis_angle), duration)
            self.interpolator_orientation = Interpolator(
                lambda t: R.from_rotvec(time_law_orientation.evaluate(t) * axis_angle / np.linalg.norm(axis_angle)).as_matrix(),
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
    time_law_type = "cubic"  # Options: "linear", "cubic", "quintic"
    controller = Controller(rospy, control_rate, time_law_type)

    rate = rospy.Rate(control_rate)

    last_time = time.time()  # Initialize the timer
    while not rospy.is_shutdown():
        current_time = time.time()
        elapsed_time = current_time - last_time
        frequency = 1.0 / elapsed_time if elapsed_time > 0 else 0  # Compute frequency
        rospy.loginfo(f"Control Loop Frequency: {frequency:.2f} Hz")
        last_time = current_time
        controller.compute_control_action()
        rate.sleep()
