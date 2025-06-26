
#include "odri_ros2_gazebo/odri_gazebo_plugin.hpp"

#include <algorithm>

#include <gazebo/physics/physics.hh>
#include <iostream>
#include <gazebo_ros/node.hpp>

namespace odri_ros2_gazebo_plugin
{

OdriGazeboPlugin::OdriGazeboPlugin()
    : robot_namespace_{""}, last_sim_time_{0}, last_update_time_{0}, update_period_ms_{1.5}
{
}

void OdriGazeboPlugin::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf)
{
    // Get model and world references
    model_ = model;
    world_ = model_->GetWorld();

    auto physicsEngine = world_->Physics();

    initializeRosObjects(sdf);
    parseSdf(sdf);
    initializeStateMachine();
    initializeDataObjects();
    printInfo();

    // Hook into simulation update loop
    update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(std::bind(&OdriGazeboPlugin::Update, this));
}

void OdriGazeboPlugin::initializeRosObjects(sdf::ElementPtr sdf)
{
    ros_node_ = gazebo_ros::Node::Get(sdf);
    RCLCPP_INFO(ros_node_->get_logger(), "Loading Odri Gazebo Plugin");

    pub_robot_state_     = ros_node_->create_publisher<odri_ros2_interfaces::msg::RobotState>("odri/robot_state", 1);
    subs_motor_commands_ = ros_node_->create_subscription<odri_ros2_interfaces::msg::RobotCommand>(
        "odri/robot_command", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile(),
        std::bind(&OdriGazeboPlugin::callbackRobotCommand, this, std::placeholders::_1));

    // StateMachine
    std::string service_name = std::string("odri/robot_interface/state_transition");
    ros_node_->declare_parameter<double>("status_pub_period", 0.2);
    status_pub_period_ = std::chrono::duration<double>(ros_node_->get_parameter("status_pub_period").as_double());

    service_sm_transition_ = ros_node_->create_service<hidro_ros2_utils::srv::TransitionCommand>(
        service_name.c_str(),
        std::bind(&OdriGazeboPlugin::transitionRequest, this, std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default);

    pub_status_ =
        ros_node_->create_publisher<hidro_ros2_utils::msg::StateMachineStatus>("odri/state_machine_status", 1);

    timer_status_pub_ = ros_node_->create_wall_timer(status_pub_period_,
                                                     std::bind(&OdriGazeboPlugin::timerPublishStateCallback, this));
}

void OdriGazeboPlugin::parseSdf(sdf::ElementPtr sdf)
{
    if (!sdf->HasElement("joint"))
    {
        RCLCPP_ERROR(ros_node_->get_logger(),
                     "Please, specify the name of the joint to attach an ODRI gazebo driver. Add <joint> tag to "
                     "your URDF plugin definition.");
    }

    sdf::ElementPtr     joint_element = sdf->GetElement("joint");
    auto                all_joints    = model_->GetJoints();
    std::vector<double> safe_positions;
    std::vector<double> safe_torques;
    while (joint_element)
    {
        std::string joint_name = joint_element->GetAttribute("name")->GetAsString();

        auto joint = model_->GetJoint(joint_name);

        if (!joint)
        {
            RCLCPP_WARN_STREAM(ros_node_->get_logger(), "Skipping joint in the URDF named '"
                                                            << joint_name << "' which is not in the gazebo model.");
            continue;
        }

        auto   param_element = joint_element->GetElement("param");
        double safe_position = 0.0;
        double safe_torque;
        while (param_element)
        {
            std::string param_name = param_element->GetAttribute("name")->GetAsString();
            if (param_name == "safe_position")
            {
                safe_position = param_element->Get<double>();
            }
            else if (param_name == "safe_torque")
            {
                safe_torque = param_element->Get<double>();
            }

            param_element = param_element->GetNextElement();
        }
        safe_positions.push_back(safe_position);
        safe_torques.push_back(safe_torque);

        joints_.push_back(joint);
        joint_names_.push_back(joint_name);
        joint_element = joint_element->GetNextElement();
    }

    if (joints_.size() == 0)
    {
        RCLCPP_ERROR(ros_node_->get_logger(), "Joint names introduced in the URDF plugin do not exist.");
    }

    safe_positions_ = Eigen::Map<Eigen::VectorXd>(safe_positions.data(), safe_positions.size());
    safe_torques_   = Eigen::Map<Eigen::VectorXd>(safe_torques.data(), safe_torques.size());
}

void OdriGazeboPlugin::initializeStateMachine()
{
    state_machine_ = hidro_utils::StateMachineDefault::create();

    state_machine_->assignTransitionCallback("enable", &OdriGazeboPlugin::transEnableCallback, this);
    state_machine_->assignTransitionCallback("start", &OdriGazeboPlugin::transStartCallback, this);
    state_machine_->assignTransitionCallback("disable", &OdriGazeboPlugin::transDisableCallback, this);
    state_machine_->assignTransitionCallback("stop", &OdriGazeboPlugin::transStopCallback, this);
}

void OdriGazeboPlugin::initializeDataObjects()
{
    const std::size_t& nj = joints_.size();
    des_torques_          = Eigen::VectorXd::Zero(nj);
    des_positions_        = Eigen::VectorXd::Zero(nj);
    des_velocities_       = Eigen::VectorXd::Zero(nj);
    des_pos_gains_        = Eigen::VectorXd::Zero(nj);
    des_vel_gains_        = Eigen::VectorXd::Zero(nj);
    max_currents_         = Eigen::VectorXd::Zero(nj);
}

void OdriGazeboPlugin::printInfo()
{
    RCLCPP_INFO(ros_node_->get_logger(), "Loaded Odri Gazebo Plugin");

    RCLCPP_INFO(ros_node_->get_logger(), "Odri Joints: ");
    for (auto j_n : joint_names_)
    {
        RCLCPP_INFO_STREAM(ros_node_->get_logger(), "\t" << j_n);
    }
    RCLCPP_INFO_STREAM(ros_node_->get_logger(), "\tSafe positions: " << safe_positions_.transpose());
    RCLCPP_INFO_STREAM(ros_node_->get_logger(), "\tSafe torques: " << safe_torques_.transpose());
}

void OdriGazeboPlugin::Update()
{
    auto cur_time = world_->SimTime();
    if (last_sim_time_ == 0)
    {
        last_sim_time_    = cur_time;
        last_update_time_ = cur_time;
        return;
    }

    auto dt = (cur_time - last_sim_time_).Double();

    // Publish joint states
    auto update_dt = (cur_time - last_update_time_).Double();
    if (update_dt * 1000 >= update_period_ms_)
    {
        robot_state_msg_.header.stamp = ros_node_->get_clock()->now();
        robot_state_msg_.motor_states.clear();

        for (std::size_t i = 0; i < joints_.size(); ++i)
        {
            auto& joint = joints_[i];

            odri_ros2_interfaces::msg::MotorState m_state;

            m_state.position                = joint->Position(0);
            m_state.velocity                = joint->GetVelocity(0);
            m_state.torque                  = joint->GetForce(0u);
            m_state.is_enabled              = true;
            m_state.has_index_been_detected = true;

            robot_state_msg_.motor_states.push_back(m_state);
        }
        pub_robot_state_->publish(robot_state_msg_);
        last_update_time_ = cur_time;
    }

    // Update control
    for (std::size_t i = 0; i < joints_.size(); ++i)
    {
        auto& joint = joints_[i];

        double force = des_torques_[i] + des_pos_gains_[i] * (des_positions_[i] - joint->Position(0)) +
                       des_vel_gains_[i] * (des_velocities_[i] - joint->GetVelocity(0));
        joint->SetForce(0, force);
    }

    last_sim_time_ = cur_time;
}

void OdriGazeboPlugin::callbackRobotCommand(const odri_ros2_interfaces::msg::RobotCommand::SharedPtr msg)
{
    int nj = std::min(msg->motor_commands.size(), joints_.size());
    if (state_machine_->getStateActive() == "running")
    {
        for (std::size_t i = 0; i < nj; ++i)
        {
            des_torques_(i)    = msg->motor_commands[i].torque_ref;
            des_positions_(i)  = msg->motor_commands[i].position_ref;
            des_velocities_(i) = msg->motor_commands[i].velocity_ref;
            des_pos_gains_(i)  = msg->motor_commands[i].kp;
            des_vel_gains_(i)  = msg->motor_commands[i].kd;
        }
    }
}

void OdriGazeboPlugin::transitionRequest(
    const std::shared_ptr<hidro_ros2_utils::srv::TransitionCommand::Request>  request,
    const std::shared_ptr<hidro_ros2_utils::srv::TransitionCommand::Response> response)
{
    RCLCPP_INFO_STREAM(ros_node_->get_logger(), "Service request received");

    std::string transition = request->command;

    hidro_utils::TransitionResponse t_response;
    state_machine_->callTransition(transition, t_response);

    response->accepted = t_response.accepted;
    response->result   = t_response.result;
    response->message  = t_response.message;

    if (!response->result)
    {
        RCLCPP_WARN_STREAM(ros_node_->get_logger(), response->message);
    }
}

void OdriGazeboPlugin::timerPublishStateCallback()
{
    state_machine_status_msg_.header.stamp = ros_node_->get_clock()->now();

    state_machine_status_msg_.status = state_machine_->getStateActive();

    pub_status_->publish(state_machine_status_msg_);
}

bool OdriGazeboPlugin::transEnableCallback(std::string& message)
{
    RCLCPP_INFO_STREAM(ros_node_->get_logger(), "\nEnable: Going to zero position");

    des_torques_   = safe_torques_;
    des_positions_ = safe_positions_;
    des_velocities_.setZero();
    des_pos_gains_.setConstant(2);
    des_vel_gains_.setConstant(0.2);

    message = "ODRI enabled successful";
    return true;
}

bool OdriGazeboPlugin::transStartCallback(std::string& message)
{
    return true;
}

bool OdriGazeboPlugin::transDisableCallback(std::string& message)
{
    des_torques_.setZero();
    des_positions_.setZero();
    des_velocities_.setZero();
    des_pos_gains_.setZero();
    des_vel_gains_.setZero();

    return true;
}

bool OdriGazeboPlugin::transStopCallback(std::string& message)
{
    des_torques_.setZero();
    des_positions_.setZero();
    des_velocities_.setZero();
    des_pos_gains_.setZero();
    des_vel_gains_.setZero();

    return true;
}

GZ_REGISTER_MODEL_PLUGIN(OdriGazeboPlugin)

}  // namespace odri_ros2_gazebo_plugin
