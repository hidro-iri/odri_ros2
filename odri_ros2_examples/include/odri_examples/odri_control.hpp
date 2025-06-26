#pragma once

// Includes Eigen library for linear algebra operations (vectors/matrices)
#include <Eigen/Dense>
// Includes ROS 2 core library for node creation and communication
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

// Includes custom ROS 2 message and service types for robot control and state
#include "odri_ros2_interfaces/msg/robot_command.hpp"
#include "odri_ros2_interfaces/msg/robot_state.hpp"
#include "odri_ros2_interfaces/srv/direct_command.hpp"
#include "odri_ros2_interfaces/srv/position_command.hpp"

// Class representing the OdriControl node, inheriting from rclcpp::Node for ROS 2 functionality
class OdriControl : public rclcpp::Node
{
public:
  // Constructor that initializes the node with a specified name
  explicit OdriControl(const std::string &node_name);

  // Destructor
  virtual ~OdriControl();

private:
  void declareAndInitializeParameters();
  rcl_interfaces::msg::SetParametersResult onParameterChange(const std::vector<rclcpp::Parameter> &parameters);
  // Timer callback function to change the robot command periodically
  void callbackTimerChangeCommand();

  // Timer callback function to publish the robot command periodically
  void callbackTimerPublishCommand();

  // Set position control mode with desired position, proportional, and derivative gains
  void setPositionControl(const Eigen::Vector2d &pos_des, const Eigen::Vector2d &kp, const Eigen::Vector2d &kd);

  // Set direct control mode (used for sending torque commands)
  void setDirectControl();

  // Set transition control mode for moving from one position to another over a specified duration
  void setTransitionControl(const Eigen::Vector2d &pos_init, const Eigen::Vector2d &pos_fin);

  // Generate position command to be sent to the robot
  void generatePositionCommand();

  // Generate direct (torque) command to be sent to the robot
  void generateDirectCommand();

  // Generate transition command for smooth trajectory generation
  void generateTransitionCommand();

  // Check if any physical limits of the robot are reached (e.g., joint limits)
  bool isLimitReached();

  // Check if the transition from one position to another is finished
  bool isTransitionFinished();

  // Callback function for receiving the robot's current state via subscription
  void callbackRobotState(const odri_ros2_interfaces::msg::RobotState::SharedPtr msg);

  // Callback for handling direct command service requests
  void callbackDirectCommand(const std::shared_ptr<odri_ros2_interfaces::srv::DirectCommand::Request> request,
                             const std::shared_ptr<odri_ros2_interfaces::srv::DirectCommand::Response> response);

  // Callback for handling position command service requests
  void callbackPositionCommand(const std::shared_ptr<odri_ros2_interfaces::srv::PositionCommand::Request> request,
                               const std::shared_ptr<odri_ros2_interfaces::srv::PositionCommand::Response> response);

private:
  // Timer for triggering command changes at fixed intervals
  rclcpp::TimerBase::SharedPtr timer_change_command_;

  // Timer for triggering command publications at fixed intervals
  rclcpp::TimerBase::SharedPtr timer_publish_command_;

  // Subscriber for receiving the robot's state
  rclcpp::Subscription<odri_ros2_interfaces::msg::RobotState>::SharedPtr sub_robot_state_;

  // Publisher for sending commands to the robot
  rclcpp::Publisher<odri_ros2_interfaces::msg::RobotCommand>::SharedPtr pub_robot_command_;

  // Command message that holds the current robot command
  odri_ros2_interfaces::msg::RobotCommand msg_robot_command_;

  rclcpp::Service<odri_ros2_interfaces::srv::DirectCommand>::SharedPtr direct_command_srv_;
  rclcpp::Service<odri_ros2_interfaces::srv::PositionCommand>::SharedPtr position_command_srv_;

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;

  // Enum representing the control mode of the robot (position, direct, or transition)
  enum class Control
  {
    position,
    direct,
    transition
  } control_type_;

  // Parameters for position control mode
  struct PositionParams
  {
    Eigen::Vector2d pos_des; // Desired position
    Eigen::Vector2d kp;      // Proportional gain
    Eigen::Vector2d kd;      // Derivative gain
  } position_params_;

  // Parameters for direct (torque-based) control mode
  struct DirectParams
  {
    Eigen::Vector2d torque_des;         // Desired torque
    Eigen::Vector2d pos_lim;            // Position limits
    Eigen::Vector2d pos_cur;            // Current position
    std::vector<bool> is_limit_reached; // Flags indicating if limits are reached
    double i_sat;
  } direct_params_;

  // Parameters for transition control mode (moving between two positions)
  struct TransitionParams
  {
    Eigen::Vector2d pos_init;               // Initial position
    Eigen::Vector2d pos_fin;                // Final position
    Eigen::Vector2d kp;                     // Proportional gain
    Eigen::Vector2d kd;                     // Derivative gain
    std::chrono::duration<double> duration; // Duration of the transition

    Eigen::Vector2d a, b, c, d;                            // Coefficients for polynomial trajectory
    std::chrono::high_resolution_clock::time_point t_init; // Start time of the transition
    bool is_first;                                         // Flag indicating if this is the first transition
  } transition_params_;
};
