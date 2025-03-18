#include "odri_examples/odri_control.hpp"
#include <cmath>

#define I_SAT 2.0

// Template function to return the sign of a value
template <typename T>
int sgn(T val)
{
  return (T(0) < val) - (val < T(0));
}

// Constructor for OdriControl node, initializes subscriptions, publishers, and timers
OdriControl::OdriControl(const std::string &node_name) : Node(node_name)
{
  // Declare and initialize parameters
  declareAndInitializeParameters();

  // Set up a parameter event callback for dynamic updates
  parameter_callback_handle_ = add_on_set_parameters_callback(
      std::bind(&OdriControl::onParameterChange, this, std::placeholders::_1));

  sub_robot_state_ = create_subscription<odri_ros2_msgs::msg::RobotState>(
      "odri/robot_state", rclcpp::SensorDataQoS(),
      std::bind(&OdriControl::callbackRobotState, this, std::placeholders::_1));

  pub_robot_command_ = create_publisher<odri_ros2_msgs::msg::RobotCommand>("odri/robot_command", 1);

  timer_change_command_ = create_wall_timer(std::chrono::duration<double, std::milli>(1),
                                            std::bind(&OdriControl::callbackTimerChangeCommand, this));

  timer_publish_command_ = create_wall_timer(std::chrono::duration<double, std::milli>(2),
                                             std::bind(&OdriControl::callbackTimerPublishCommand, this));

  direct_command_srv_ = create_service<odri_ros2_msgs::srv::DirectCommand>(
      "direct_command", std::bind(&OdriControl::callbackDirectCommand, this, std::placeholders::_1, std::placeholders::_2));

  position_command_srv_ = create_service<odri_ros2_msgs::srv::PositionCommand>(
      "position_command", std::bind(&OdriControl::callbackPositionCommand, this, std::placeholders::_1, std::placeholders::_2));

  // Initialize control type
  control_type_ = Control::position;
  std::cout << "Position command\n";

  // Initialize direct params
  direct_params_.is_limit_reached = std::vector<bool>(2, false);
  direct_params_.i_sat = I_SAT;

  // Initialize transition params
  transition_params_.is_first = true;
  transition_params_.a.setZero();
  transition_params_.b.setZero();
  transition_params_.c.setZero();
  transition_params_.d.setZero();

  // Initialize message
  msg_robot_command_ = odri_ros2_msgs::msg::RobotCommand();
}

// Destructor
OdriControl::~OdriControl() {}

void OdriControl::declareAndInitializeParameters()
{
  // Declare parameters for kp and kd as vectors
  declare_parameter<std::vector<double>>("transition_kp", {2., 2.});   // Initial values for kp_x and kp_y
  declare_parameter<std::vector<double>>("transition_kd", {0.3, 0.3}); // Initial values for kd_x and kd_y
  declare_parameter<double>("transition_duration", 2.0);               // in seconds
  declare_parameter<double>("direct_i_sat", 2.0);                      // in seconds

  // Get initial values of parameters
  std::vector<double> kp_vec = get_parameter("transition_kp").as_double_array();
  std::vector<double> kd_vec = get_parameter("transition_kd").as_double_array();

  // Initialize control structures
  transition_params_.kp = Eigen::Vector2d(kp_vec.data());
  transition_params_.kd = Eigen::Vector2d(kd_vec.data());
  transition_params_.duration = std::chrono::duration<double>(this->get_parameter("transition_duration").as_double());
  direct_params_.i_sat = this->get_parameter("direct_i_sat").as_double();
}

rcl_interfaces::msg::SetParametersResult OdriControl::onParameterChange(const std::vector<rclcpp::Parameter> &parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  for (const auto &param : parameters)
  {
    if (param.get_name() == "transition_duration")
    {
      // Update transition duration at runtime
      transition_params_.duration = std::chrono::duration<double>(param.as_double());
    }
    if (param.get_name() == "direct_i_sat")
    {
      direct_params_.i_sat = param.as_double();
    }
    else if (param.get_name() == "transition_kp")
    {
      std::vector<double> kp_vec = param.as_double_array();
      transition_params_.kp = Eigen::Vector2d(kp_vec.data());
    }
    else if (param.get_name() == "transition_kd")
    {
      std::vector<double> kd_vec = param.as_double_array();
      transition_params_.kd = Eigen::Vector2d(kd_vec.data());
    }
  }

  return result;
}

// Timer callback to change control mode based on limits or transitions
void OdriControl::callbackTimerChangeCommand()
{
  switch (control_type_)
  {
  case Control::direct:
    if (isLimitReached())
    {
      setTransitionControl(direct_params_.pos_cur, position_params_.pos_des);
      control_type_ = Control::transition;
      std::cout << "Transition command\n";
    };
    break;
  case Control::transition:
    if (isTransitionFinished())
    {
      setPositionControl(transition_params_.pos_fin, position_params_.kp, position_params_.kd);
      control_type_ = Control::position;
      std::cout << "Position command\n";
    };
    break;
  default:
    return;
    break;
  }
}

// Check if limits are reached in direct control
bool OdriControl::isLimitReached()
{
  bool is_limit_reached = true;
  for (std::size_t i = 0; i < 2; ++i)
  {
    if (!direct_params_.is_limit_reached.at(i))
    {
      int sign_torque = sgn(direct_params_.torque_des(i));
      if (sign_torque * (direct_params_.pos_lim(i) - direct_params_.pos_cur(i)) < 0)
      {
        direct_params_.is_limit_reached.at(i) = true;
      }
      else
      {
        is_limit_reached = false;
      }
    }
  }
  return is_limit_reached;
}

// Check if the transition is complete
bool OdriControl::isTransitionFinished()
{
  if (!transition_params_.is_first)
  {
    auto t = std::chrono::system_clock::now();
    double dt = std::chrono::duration_cast<std::chrono::duration<double>>(t - transition_params_.t_init).count();
    if (transition_params_.duration.count() < dt)
    {
      return true;
    }
  }
  return false;
}

// Set the robot to position control mode
void OdriControl::setPositionControl(const Eigen::Vector2d &pos_des, const Eigen::Vector2d &kp, const Eigen::Vector2d &kd)
{
  position_params_.pos_des = pos_des;
  position_params_.kp = kp;
  position_params_.kd = kd;
}

// Set transition control for smooth movement between two positions
void OdriControl::setTransitionControl(const Eigen::Vector2d &pos_init, const Eigen::Vector2d &pos_fin)
{
  transition_params_.pos_init = pos_init;
  transition_params_.pos_fin = pos_fin;
  double dt = transition_params_.duration.count();
  for (std::size_t i = 0; i < 2; ++i)
  {
    double d_pose = pos_fin(i) - pos_init(i);
    transition_params_.d(i) = pos_init(i);
    transition_params_.c(i) = 0.0;
    transition_params_.b(i) = (6 * d_pose) / (dt * dt);
    transition_params_.a(i) = -transition_params_.b(i) / dt;
  }
  transition_params_.t_init = std::chrono::system_clock::now();
  transition_params_.is_first = true;
}

// Timer callback to publish robot commands based on the current control mode
void OdriControl::callbackTimerPublishCommand()
{
  msg_robot_command_.header.stamp = get_clock()->now();
  msg_robot_command_.motor_commands.clear();

  switch (control_type_)
  {
  case Control::position:
    generatePositionCommand();
    break;
  case Control::direct:
    generateDirectCommand();
    break;
  case Control::transition:
    generateTransitionCommand();
    break;
  default:
    return;
    break;
  }
  pub_robot_command_->publish(msg_robot_command_);
}

// Callback for robot state subscription
void OdriControl::callbackRobotState(const odri_ros2_msgs::msg::RobotState::SharedPtr msg)
{
  for (std::size_t i = 0; i < 2; ++i)
  {
    direct_params_.pos_cur(i) = msg->motor_states[i].position;
  }
}

// Callback for handling direct command requests
void OdriControl::callbackDirectCommand(const std::shared_ptr<odri_ros2_msgs::srv::DirectCommand::Request> request,
                                        const std::shared_ptr<odri_ros2_msgs::srv::DirectCommand::Response> response)
{
  for (std::size_t i = 0; i < 2; ++i)
  {
    direct_params_.torque_des(i) = request->torque_des[i];
    direct_params_.pos_lim(i) = request->pos_lim[i];
    direct_params_.is_limit_reached.at(i) = false;
  }
  control_type_ = Control::direct;
  std::cout << "Direct command\n";

  response->accepted = true;
}

// Callback for handling position command requests
void OdriControl::callbackPositionCommand(const std::shared_ptr<odri_ros2_msgs::srv::PositionCommand::Request> request,
                                          const std::shared_ptr<odri_ros2_msgs::srv::PositionCommand::Response> response)
{
  for (std::size_t i = 0; i < 2; ++i)
  {
    position_params_.pos_des(i) = request->pos_des[i];
    position_params_.kp(i) = request->kp[i];
    position_params_.kd(i) = request->kd[i];
  }
  setTransitionControl(direct_params_.pos_cur, position_params_.pos_des);
  control_type_ = Control::transition;
  std::cout << "Transition command\n";

  response->accepted = true;
}

// Generate the position control command to send to the robot
void OdriControl::generatePositionCommand()
{
  for (std::size_t i = 0; i < 2; ++i)
  {
    odri_ros2_msgs::msg::MotorCommand command;

    command.position_ref = position_params_.pos_des(i);
    command.velocity_ref = 0.0;
    command.torque_ref = 0.0;
    command.kp = position_params_.kp(i);
    command.kd = position_params_.kd(i);
    command.i_sat = direct_params_.i_sat;
    msg_robot_command_.motor_commands.push_back(command);
  }
}

// Generate the direct (torque) control command
void OdriControl::generateDirectCommand()
{
  for (std::size_t i = 0; i < 2; ++i)
  {
    odri_ros2_msgs::msg::MotorCommand command;

    if (!direct_params_.is_limit_reached.at(i))
    {
      command.position_ref = 0.0;
      command.velocity_ref = 0.0;
      command.torque_ref = direct_params_.torque_des(i);
      command.kp = 0.0;
      command.kd = 0.0;
    }
    else
    {
      command.position_ref = direct_params_.pos_lim(i);
      command.velocity_ref = 0.0;
      command.torque_ref = 0.0;
      command.kp = transition_params_.kp(i);
      command.kd = transition_params_.kd(i);
    }
    command.i_sat = direct_params_.i_sat;
    msg_robot_command_.motor_commands.push_back(command);
  }
}

// Generate the transition command for smooth position control
void OdriControl::generateTransitionCommand()
{
  auto t = std::chrono::system_clock::now();
  if (transition_params_.is_first)
  {
    transition_params_.t_init = t;
    transition_params_.is_first = false;
  }
  std::chrono::duration<double> duration = std::chrono::duration_cast<std::chrono::duration<double>>(t - transition_params_.t_init);
  double dt = std::min(duration.count(), transition_params_.duration.count());
  for (std::size_t i = 0; i < 2; ++i)
  {
    odri_ros2_msgs::msg::MotorCommand command;

    command.position_ref = transition_params_.d(i) + dt * (transition_params_.c(i) + dt * (transition_params_.b(i) / 2 + dt * transition_params_.a(i) / 3.));
    command.velocity_ref = transition_params_.c(i) + dt * (transition_params_.b(i) + dt * transition_params_.a(i));
    command.torque_ref = 0.0;
    command.kp = transition_params_.kp(i);
    command.kd = transition_params_.kd(i);
    command.i_sat = direct_params_.i_sat;
    msg_robot_command_.motor_commands.push_back(command);
  }
}

// Main function
int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  std::shared_ptr<OdriControl> odri_controller =
      std::make_shared<OdriControl>("odri_controller");

  rclcpp::executors::MultiThreadedExecutor executor;
  // rclcpp::executors::StaticSingleThreadedExecutor executor;
  executor.add_node(odri_controller);

  executor.spin();
  rclcpp::shutdown();

  return 0;
}
