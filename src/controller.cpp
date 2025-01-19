#include "../include/controller.hpp"
#include <chrono>
#include <thread>

/**
 * @brief Compute the orientation error between two orientations.
 *
 * This function computes the orientation error between a desired and current rotation matrix orientation.
 *
 * @param desired The desired orientation in quaternion form.
 * @param current The current orientation in quaternion form.
 * @return Eigen::Vector3d The orientation error.
 */
Eigen::Vector3d Controller::ComputeOrientationError(Eigen::Matrix3d& tcp_orientation,
                                                    Eigen::Matrix3d& desired_orientation)
{
  Eigen::Matrix3d delta_R = desired_orientation * tcp_orientation.transpose();
  Eigen::Quaterniond delta_q = Eigen::Quaterniond(delta_R);
  Eigen::Quaterniond delta_q_negated = Eigen::Quaterniond(-delta_q.w(), -delta_q.x(), -delta_q.y(), -delta_q.z());

  double angle_delta_q = 2 * acos(delta_q.w());
  double angle_delta_q_negated = 2 * acos(delta_q_negated.w());

  if (abs(angle_delta_q) <= abs(angle_delta_q_negated))
    delta_q = delta_q;
  else
    delta_q = delta_q_negated;

  Eigen::Vector3d orientation_error = Eigen::Vector3d(delta_q.x(), delta_q.y(), delta_q.z());

  return orientation_error;
}

// ----------------------------------------------------------------- //
// --------------------------- Callbacks --------------------------- //
// ----------------------------------------------------------------- //

void Controller::DesiredPoseCallback(const geometry_msgs::PoseStamped& msg)
{
  auto now = std::chrono::high_resolution_clock::now();
  double delta_time = std::chrono::duration_cast<std::chrono::microseconds>(now - this->prev_time).count() / 1000000.0;
  // ros::Time now = msg.header.stamp;
  // double delta_time = (now - this->prev_time).toSec();

  is_pose_stream_active = true; // New pose received

  Eigen::Affine3d desired_pose;
  Eigen::fromMsg(msg.pose, desired_pose);
  Eigen::Vector3d tcp_increment = desired_pose.translation();
  Eigen::Vector3d desired_position = this->initial_tcp_position + tcp_increment;
  Eigen::Vector3d desired_position_prev = this->desired_position_prev;
  
  Eigen::Vector3d desired_vel = (desired_position - this->desired_position_prev) / delta_time;

  // Orientation part
  Eigen::Matrix3d desired_R = desired_pose.rotation();
  Eigen::Matrix3d desired_R_prev = this->desired_R_prev;

  Eigen::Matrix3d delta_R = desired_R_prev.transpose() * desired_R;
  double desired_angle = Eigen::AngleAxisd(delta_R).angle();
  double desired_omega = desired_angle / delta_time;

  // Interpolator
  // Position part
  // TimeLawShrPtr pTimeLawPosition = std::make_shared<QuinticTimeLaw>(desired_position, desired_position_prev, desired_vel, desired_vel_prev, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), delta_time); 
  // TimeLawShrPtr pTimeLawPosition = std::make_shared<CubicalTimeLaw>(desired_position, desired_position_prev, desired_vel, desired_vel_prev, delta_time);
  TimeLawShrPtr pTimeLawPosition = std::make_shared<LinearTimeLaw>(desired_position, desired_position_prev, delta_time);
  auto positionPathFunction = [desired_position, desired_position_prev, pTimeLawPosition](double time) -> Eigen::Vector3d {
    Eigen::Vector3d diff = desired_position - desired_position_prev;
    Eigen::Vector3d p;
    if (diff.norm() == 0)
      p = desired_position;
    else
      p = desired_position_prev + pTimeLawPosition->evaluate(time) * (diff) / (diff).norm();
    return p;
  };

  this->interpolator_position = Interpolator<Eigen::Vector3d>(positionPathFunction, delta_time);
  auto position_result = this->interpolator_position.discretiseTrajectory(this->robot_loop_rate, this->start_time);
  std::list<Eigen::Vector3d> new_positions = std::get<0>(position_result);
  double position_start_time = 1.0/this->robot_loop_rate - std::get<1>(position_result);
  this->buffered_position.insert(this->buffered_position.end(), new_positions.begin(), new_positions.end());

  // Orientation part
  Eigen::Vector3d desired_axis = Eigen::AngleAxisd(delta_R).axis();

  // TimeLawShrPtr pTimeLawOrientation = std::make_shared<QuinticTimeLaw>(desired_angle, 0.0, desired_omega, desired_omega_prev, 0.0, 0.0, delta_time); 
  // TimeLawShrPtr pTimeLawOrientation = std::make_shared<CubicalTimeLaw>(desired_angle, 0.0, desired_omega, desired_omega_prev, delta_time);
  TimeLawShrPtr pTimeLawOrientation = std::make_shared<LinearTimeLaw>(desired_angle, 0.0, delta_time);
  auto orientationPathFunction = [desired_R_prev, desired_angle, desired_axis, desired_omega, pTimeLawOrientation](double time) -> Eigen::Matrix3d {
    double current_angle;
    if (desired_angle == 0)
      current_angle = desired_angle;
    else
      current_angle = 0 + pTimeLawOrientation->evaluate(time) * (desired_angle - 0) / (desired_angle - 0);

    Eigen::AngleAxisd axis_angle = Eigen::AngleAxisd(current_angle, desired_axis);

    return desired_R_prev * axis_angle.matrix();
  };

  this->interpolator_orientation = Interpolator<Eigen::Matrix3d>(orientationPathFunction, delta_time);
  auto orientation_result = this->interpolator_orientation.discretiseTrajectory(this->robot_loop_rate, this->start_time);
  std::list<Eigen::Matrix3d> new_orientation = std::get<0>(orientation_result);
  double orientation_start_time = 1.0/this->robot_loop_rate - std::get<1>(orientation_result);
  this->buffered_orientation.insert(this->buffered_orientation.end(), new_orientation.begin(), new_orientation.end());

  // Updating desired position, orientation, velocity, angular velocity and time_prev
  this->desired_position_prev = desired_position;
  this->desired_R_prev = desired_R;
  this->desired_vel_prev = desired_vel;
  this->desired_omega_prev = desired_omega;
  this->prev_time = now;
  this->start_time = position_start_time;

  return;
}

void Controller::JointStateCallback(const sensor_msgs::JointState& msg)
{
    // List of desired joint names
    std::vector<std::string> joint_names = { "shoulder_pan_joint", "shoulder_lift_joint", 
                                             "elbow_joint", "wrist_1_joint", 
                                             "wrist_2_joint", "wrist_3_joint"};

    // Map to store the correspondence between joint names and indices in msg.position
    std::map<std::string, size_t> joint_index_map;

    // Build the map based on msg.name
    for (size_t i = 0; i < msg.name.size(); ++i)
    {
        joint_index_map[msg.name[i]] = i;
    }

    // Populate joint_position using the map
    for (size_t i = 0; i < joint_names.size(); ++i)
    {
        auto it = joint_index_map.find(joint_names[i]);
        if (it != joint_index_map.end())
        {
            this->joint_position[i] = msg.position[it->second];
        }
        else
        {
            // Handle the case where the joint name is not found in the message
            ROS_WARN_STREAM("Joint " << joint_names[i] << " not found in JointState message.");
        }
    }
}

void Controller::PublishZeroVelocities()
{
    std_msgs::Float64MultiArray zero_vel;
    zero_vel.data = std::vector<double>(6, 0.0); // Assuming 6 DOF
    joint_velocity_pub.publish(zero_vel);
    ROS_INFO("Published zero velocities to stop the robot.");
}


void Controller::ComputeControlAction()
{
  // Getting feedback
  Eigen::Affine3d tcp_pose;
  if( this->p_robot_kinematics->computeFk(this->joint_position, tcp_pose) == false ) 
    return;
  this->tcp_position = tcp_pose.translation();
  this->tcp_orientation = tcp_pose.rotation();

  // ---------------- Debug topics ---------------- //
  // feedback
  geometry_msgs::Pose feedback;
  feedback.position.x = this->tcp_position.x();
  feedback.position.y = this->tcp_position.y();
  feedback.position.z = this->tcp_position.z();
  feedback_pub.publish(feedback);

  // ---------------- Debug topics ---------------- //
  // Check if pose stream has ended and buffers are empty
  if (!is_pose_stream_active && buffered_position.empty() && buffered_orientation.empty())
  {
      ROS_INFO("Control actions stopped: No new poses and buffers are empty.");
      PublishZeroVelocities(); // Ensure robot stops moving
      return; // Stop control computation
  }

  // Reset pose stream flag to detect inactivity
  is_pose_stream_active = false;

  if (this->buffered_position.empty() || this->buffered_orientation.empty())
    return;

  Eigen::Vector3d setpoint_position = this->buffered_position.front();
  Eigen::Matrix3d setpoint_orientation = this->buffered_orientation.front();
  this->buffered_position.pop_front();
  this->buffered_orientation.pop_front();

  // ---------------------------------------
  // Computing error
  Eigen::Vector3d position_error = setpoint_position - this->tcp_position;
  Eigen::Vector3d orientation_error = ComputeOrientationError(this->tcp_orientation, setpoint_orientation);

  if (std::isnan(position_error[0]) || std::isnan(position_error[1]) || std::isnan(position_error[2]) ||
      std::isnan(orientation_error[0]) || std::isnan(orientation_error[1]) || std::isnan(orientation_error[2]))
    return;

  if (position_error.norm() == 0.0 && orientation_error.norm() == 0)
    return;

  // Computing control action
  Eigen::Vector3d control_input_p = Kp.cwiseProduct(position_error);
  Eigen::Vector3d control_input_o = Ko.cwiseProduct(orientation_error);
  Vector6d control_input;
  control_input.block<3, 1>(0, 0) = control_input_p;
  control_input.block<3, 1>(3, 0) = control_input_o;

  Matrix6d Jg;
  if(this->p_robot_kinematics->computeJac(this->joint_position, Jg) == false)
    return;

  if (Jg.determinant() == 0 || Jg.isZero())
    return;
  Vector6d joint_velocities = Jg.inverse() * control_input;

  // Changing datatype
  std_msgs::Float64MultiArray q_dot;
  q_dot.data.reserve(6);
  q_dot.data.push_back(joint_velocities[0]);
  q_dot.data.push_back(joint_velocities[1]);
  q_dot.data.push_back(joint_velocities[2]);
  q_dot.data.push_back(joint_velocities[3]);
  q_dot.data.push_back(joint_velocities[4]);
  q_dot.data.push_back(joint_velocities[5]);

  // ---------------- Debug topics ---------------- //
  // error
  std_msgs::Float64MultiArray p_err;
  p_err.data.reserve(3);
  p_err.data.push_back(position_error[0]);
  p_err.data.push_back(position_error[1]);
  p_err.data.push_back(position_error[2]);
  position_error_pub.publish(p_err);
  // ---------------- Debug topics ---------------- //


  // Publish the values of the joint velocities
  joint_velocity_pub.publish(q_dot);
}

// ----------------------------------------------------------------- //
// -------------------------- Constructor -------------------------- //
// ----------------------------------------------------------------- //

Controller::Controller(ros::NodeHandle& nh, double control_loop_rate)
{
  ROS_INFO("Checkpoint 1."); 
  this->nh = nh;

  this->tf_reference_name = "base_link";                                    
  this->tf_tcp_name = "tool0";

  std::string robot_description;
  this->nh.param("robot_description", robot_description, std::string());
  this->p_robot_kinematics = std::make_shared<Kinematics>(robot_description);
  this->p_robot_kinematics->setChain(this->tf_reference_name, this->tf_tcp_name);

  // Loop rate
  this->robot_loop_rate = control_loop_rate;  // Robot update loop (Hz) // TODO: parametrise

  // Publisher
  std::string topic_input_commands_name = "/joint_group_vel_controller/command";  // TODO: parametrise
  this->joint_velocity_pub = nh.advertise<std_msgs::Float64MultiArray>(topic_input_commands_name, 1);
  this->desired_pose_interpolated_pub = nh.advertise<geometry_msgs::Pose>("desired_pose_interpolated", 1);
  this->feedback_pub = nh.advertise<geometry_msgs::Pose>("feedback", 1);
  this->position_error_pub = nh.advertise<std_msgs::Float64MultiArray>("position_error", 1);
  ROS_INFO("Checkpoint 2."); 
  // Subscriber
  this->desired_pose_sub = nh.subscribe("/generate_motion_service_node/cartesian_path", 1, &Controller::DesiredPoseCallback, this);
  this->joint_states_sub = nh.subscribe("joint_states", 1, &Controller::JointStateCallback, this);

  // Getting first msg for desired pose
  boost::shared_ptr<geometry_msgs::PoseStamped const> sharedPtrDesiredPose = NULL;
  while (sharedPtrDesiredPose == NULL)
  {
    sharedPtrDesiredPose = ros::topic::waitForMessage<geometry_msgs::PoseStamped>("/generate_motion_service_node/cartesian_path", ros::Duration(5));
    ROS_INFO("No desired_pose messages received");
  }
  ROS_INFO("Checkpoint 3."); 
  Eigen::Affine3d desired_pose;
  Eigen::fromMsg(sharedPtrDesiredPose->pose, desired_pose);
  this->desired_position_prev = desired_pose.translation();
  this->desired_R_prev = desired_pose.rotation();
  ROS_INFO("Checkpoint 4."); 
  // Waiting for first msg for joint states
  boost::shared_ptr<sensor_msgs::JointState const> sharedPtrJointStates = NULL;
  while (sharedPtrJointStates == NULL)
  {
    sharedPtrJointStates = ros::topic::waitForMessage<sensor_msgs::JointState>("/joint_states", ros::Duration(1000));
  }
  this->JointStateCallback(*sharedPtrJointStates);

  // Add this block to calculate the initial TCP position
  ROS_INFO("Checkpoint 5."); 
  Eigen::Affine3d tcp_pose;

  // New checks
  ROS_INFO_STREAM("Joint positions: " << this->joint_position.transpose());
  if (!this->p_robot_kinematics)
  {
      ROS_ERROR("Kinematics object is not initialized. Aborting.");
      return;
  }
  //

  if (this->p_robot_kinematics->computeFk(this->joint_position, tcp_pose))
  {
    this->initial_tcp_position = tcp_pose.translation();
    ROS_INFO_STREAM("Initial TCP position set to: " << this->initial_tcp_position.transpose());
  }
  else
  {
    ROS_ERROR("Failed to compute initial TCP position.");
  }
  ROS_INFO("Checkpoint 6."); 
  // Creating the TF listener
  // this->pTfListener = new tf2_ros::TransformListener(this->tfBuffer);
  // Getting the controller gains from param file
  this->Kp = Eigen::Vector3d(10, 10, 10);  // TODO: Parametrise
  this->Ko = Eigen::Vector3d(10, 10, 10);  // TODO: Parametrise

  ROS_INFO("Checkpoint 7.");

  // Setting time
  this->prev_time = std::chrono::high_resolution_clock::now();
  // this->prev_time = ros::Time::now();
  this->buffered_position = std::list<Eigen::Vector3d>();
  this->buffered_orientation = std::list<Eigen::Matrix3d>();
  ROS_INFO("Checkpoint 8."); 
  // --- TRY --- //

  // --- TRY --- //
}

