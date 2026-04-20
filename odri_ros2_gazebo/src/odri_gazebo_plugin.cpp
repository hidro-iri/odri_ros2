
#include "odri_ros2_gazebo/odri_gazebo_plugin.hpp"

#include <algorithm>
#include <iostream>

#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz/sim/components/JointForce.hh>
#include <gz/sim/components/JointForceCmd.hh>
#include <gz/plugin/Register.hh>

namespace odri_ros2_gazebo_plugin
{

OdriGazeboPlugin::OdriGazeboPlugin()
    : robot_namespace_{""}, last_sim_time_{0.0}, last_update_time_{0.0}, update_period_ms_{1.5}
{
}

OdriGazeboPlugin::~OdriGazeboPlugin()
{
    if (executor_) executor_->cancel();
    if (executor_thread_.joinable()) executor_thread_.join();
}

void OdriGazeboPlugin::Configure(const gz::sim::Entity &entity,
                                  const std::shared_ptr<const sdf::Element> &sdf,
                                  gz::sim::EntityComponentManager &ecm,
                                  gz::sim::EventManager & /*eventMgr*/)
{
    model_entity_ = entity;
    model_        = gz::sim::Model(entity);

    initializeRosObjects(sdf);
    parseSdf(sdf, ecm);
    initializeStateMachine();
    initializeDataObjects();
    printInfo();
}

void OdriGazeboPlugin::initializeRosObjects(const std::shared_ptr<const sdf::Element> &sdf)
{
    if (!rclcpp::ok())
    {
        rclcpp::init(0, nullptr);
    }

    std::string node_name = "odri_gazebo_plugin";
    if (sdf->HasElement("robotNamespace"))
    {
        robot_namespace_ = sdf->Get<std::string>("robotNamespace");
        std::string sanitized = robot_namespace_;
        std::replace(sanitized.begin(), sanitized.end(), '/', '_');
        if (!sanitized.empty() && sanitized[0] == '_')
            sanitized = sanitized.substr(1);
        node_name = "odri_gazebo_plugin_" + sanitized;
    }

    ros_node_ = std::make_shared<rclcpp::Node>(node_name);

    RCLCPP_INFO(ros_node_->get_logger(), "Loading Odri Gazebo Plugin");

    pub_robot_state_     = ros_node_->create_publisher<odri_ros2_interfaces::msg::RobotState>("odri/robot_state", 1);
    subs_motor_commands_ = ros_node_->create_subscription<odri_ros2_interfaces::msg::RobotCommand>(
        "odri/robot_command", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile(),
        std::bind(&OdriGazeboPlugin::callbackRobotCommand, this, std::placeholders::_1));

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

    executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    executor_->add_node(ros_node_);
    executor_thread_ = std::thread([this]() { executor_->spin(); });
}

void OdriGazeboPlugin::parseSdf(const std::shared_ptr<const sdf::Element> &sdf,
                                 gz::sim::EntityComponentManager &ecm)
{
    if (!sdf->HasElement("joint"))
    {
        RCLCPP_ERROR(ros_node_->get_logger(),
                     "Please, specify the name of the joint to attach an ODRI gazebo driver. Add <joint> tag to "
                     "your URDF plugin definition.");
        return;
    }

    sdf::ElementPtr sdf_clone = sdf->Clone();

    sdf::ElementPtr     joint_element = sdf_clone->GetElement("joint");
    std::vector<double> safe_positions;
    std::vector<double> safe_torques;

    while (joint_element)
    {
        std::string joint_name = joint_element->GetAttribute("name")->GetAsString();

        gz::sim::Entity joint_entity = model_.JointByName(ecm, joint_name);

        if (joint_entity == gz::sim::kNullEntity)
        {
            RCLCPP_WARN_STREAM(ros_node_->get_logger(), "Skipping joint in the URDF named '"
                                                            << joint_name << "' which is not in the gz model.");
            joint_element = joint_element->GetNextElement("joint");
            continue;
        }

        gz::sim::Joint joint(joint_entity);
        joint.EnablePositionCheck(ecm, true);
        joint.EnableVelocityCheck(ecm, true);

        auto   param_element = joint_element->HasElement("param") ? joint_element->GetElement("param") : nullptr;
        double safe_position = 0.0;
        double safe_torque   = 0.0;
        while (param_element)
        {
            std::string param_name = param_element->GetAttribute("name")->GetAsString();
            if (param_name == "safe_position")
                safe_position = param_element->Get<double>();
            else if (param_name == "safe_torque")
                safe_torque = param_element->Get<double>();
            param_element = param_element->GetNextElement("param");
        }
        safe_positions.push_back(safe_position);
        safe_torques.push_back(safe_torque);

        joint_entities_.push_back(joint_entity);
        joint_names_.push_back(joint_name);

        joint_element = joint_element->GetNextElement("joint");
    }

    if (joint_entities_.empty())
        RCLCPP_ERROR(ros_node_->get_logger(), "Joint names introduced in the URDF plugin do not exist.");

    safe_positions_ = Eigen::Map<Eigen::VectorXd>(safe_positions.data(), safe_positions.size());
    safe_torques_   = Eigen::Map<Eigen::VectorXd>(safe_torques.data(), safe_torques.size());
}

void OdriGazeboPlugin::initializeStateMachine()
{
    state_machine_ = hidro_utils::StateMachineDefault::create();

    state_machine_->assignTransitionCallback("enable",  &OdriGazeboPlugin::transEnableCallback,  this);
    state_machine_->assignTransitionCallback("start",   &OdriGazeboPlugin::transStartCallback,   this);
    state_machine_->assignTransitionCallback("disable", &OdriGazeboPlugin::transDisableCallback, this);
    state_machine_->assignTransitionCallback("stop",    &OdriGazeboPlugin::transStopCallback,    this);
}

void OdriGazeboPlugin::initializeDataObjects()
{
    const std::size_t nj = joint_entities_.size();
    des_torques_    = Eigen::VectorXd::Zero(nj);
    des_positions_  = safe_positions_;
    des_velocities_ = Eigen::VectorXd::Zero(nj);
    des_pos_gains_  = Eigen::VectorXd::Zero(nj);
    des_vel_gains_  = Eigen::VectorXd::Zero(nj);
    max_currents_   = Eigen::VectorXd::Zero(nj);
}

void OdriGazeboPlugin::printInfo()
{
    RCLCPP_INFO(ros_node_->get_logger(), "Loaded Odri Gazebo Plugin");
    RCLCPP_INFO(ros_node_->get_logger(), "Odri Joints:");
    for (const auto& j_n : joint_names_)
        RCLCPP_INFO_STREAM(ros_node_->get_logger(), "\t" << j_n);
    RCLCPP_INFO_STREAM(ros_node_->get_logger(), "\tSafe positions: " << safe_positions_.transpose());
    RCLCPP_INFO_STREAM(ros_node_->get_logger(), "\tSafe torques:   " << safe_torques_.transpose());
    RCLCPP_INFO_STREAM(ros_node_->get_logger(), "\tKp=" << kDefaultKp << "  Kd=" << kDefaultKd);
}

void OdriGazeboPlugin::PreUpdate(const gz::sim::UpdateInfo &info,
                                  gz::sim::EntityComponentManager &ecm)
{
    if (info.paused) return;

    double cur_time = std::chrono::duration<double>(info.simTime).count();

    if (last_sim_time_ == 0.0)
    {
        last_sim_time_    = cur_time;
        last_update_time_ = cur_time;
        return;
    }

    Eigen::VectorXd torques, positions, velocities, pos_gains, vel_gains;
    {
        std::lock_guard<std::mutex> lock(cmd_mutex_);
        torques    = des_torques_;
        positions  = des_positions_;
        velocities = des_velocities_;
        pos_gains  = des_pos_gains_;
        vel_gains  = des_vel_gains_;
    }

    for (std::size_t i = 0; i < joint_entities_.size(); ++i)
    {
        gz::sim::Joint joint(joint_entities_[i]);

        auto pos_opt = joint.Position(ecm);
        auto vel_opt = joint.Velocity(ecm);

        double pos = (pos_opt && !pos_opt->empty()) ? (*pos_opt)[0] : 0.0;
        double vel = (vel_opt && !vel_opt->empty()) ? (*vel_opt)[0] : 0.0;

        double force = torques[i] + pos_gains[i] * (positions[i] - pos) +
                       vel_gains[i] * (velocities[i] - vel);

        joint.SetForce(ecm, {force});
    }

    last_sim_time_ = cur_time;
}

void OdriGazeboPlugin::PostUpdate(const gz::sim::UpdateInfo &info,
                                   const gz::sim::EntityComponentManager &ecm)
{
    if (info.paused) return;

    double cur_time  = std::chrono::duration<double>(info.simTime).count();
    double update_dt = cur_time - last_update_time_;

    if (update_dt * 1000.0 >= update_period_ms_)
    {
        robot_state_msg_.header.stamp = ros_node_->get_clock()->now();
        robot_state_msg_.motor_states.clear();

        for (std::size_t i = 0; i < joint_entities_.size(); ++i)
        {
            gz::sim::Joint joint(joint_entities_[i]);

            auto pos_opt   = joint.Position(ecm);
            auto vel_opt   = joint.Velocity(ecm);
            auto *force_cmd = ecm.Component<gz::sim::components::JointForceCmd>(joint_entities_[i]);

            odri_ros2_interfaces::msg::MotorState m_state;
            m_state.position                = (pos_opt    && !pos_opt->empty())         ? (*pos_opt)[0]           : 0.0;
            m_state.velocity                = (vel_opt    && !vel_opt->empty())         ? (*vel_opt)[0]           : 0.0;
            m_state.torque                  = (force_cmd  && !force_cmd->Data().empty()) ? force_cmd->Data()[0]   : 0.0;
            m_state.is_enabled              = true;
            m_state.has_index_been_detected = true;

            robot_state_msg_.motor_states.push_back(m_state);
        }
        pub_robot_state_->publish(robot_state_msg_);
        last_update_time_ = cur_time;
    }
}

void OdriGazeboPlugin::callbackRobotCommand(const odri_ros2_interfaces::msg::RobotCommand::SharedPtr msg)
{
    std::size_t nj = std::min(msg->motor_commands.size(), joint_entities_.size());
    if (state_machine_->getStateActive() == "running")
    {
        std::lock_guard<std::mutex> lock(cmd_mutex_);
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

    hidro_utils::TransitionResponse t_response;
    state_machine_->callTransition(request->command, t_response);

    response->accepted = t_response.accepted;
    response->result   = t_response.result;
    response->message  = t_response.message;

    if (!response->result)
        RCLCPP_WARN_STREAM(ros_node_->get_logger(), response->message);
}

void OdriGazeboPlugin::timerPublishStateCallback()
{
    state_machine_status_msg_.header.stamp = ros_node_->get_clock()->now();
    state_machine_status_msg_.status       = state_machine_->getStateActive();
    pub_status_->publish(state_machine_status_msg_);
}

bool OdriGazeboPlugin::transEnableCallback(std::string& message)
{
    RCLCPP_INFO_STREAM(ros_node_->get_logger(),
                       "Enable: Kp=" << kDefaultKp << " Kd=" << kDefaultKd);
    {
        std::lock_guard<std::mutex> lock(cmd_mutex_);
        des_torques_   = safe_torques_;
        des_positions_ = safe_positions_;
        des_velocities_.setZero();
        des_pos_gains_.setConstant(kDefaultKp);
        des_vel_gains_.setConstant(kDefaultKd);
    }
    message = "ODRI enabled";
    return true;
}

bool OdriGazeboPlugin::transStartCallback(std::string& /*message*/)
{
    return true;
}

bool OdriGazeboPlugin::transDisableCallback(std::string& /*message*/)
{
    RCLCPP_INFO(ros_node_->get_logger(), "Disable: zero forces");
    std::lock_guard<std::mutex> lock(cmd_mutex_);
    des_torques_.setZero();
    des_positions_.setZero();
    des_velocities_.setZero();
    des_pos_gains_.setZero();
    des_vel_gains_.setZero();
    return true;
}

bool OdriGazeboPlugin::transStopCallback(std::string& /*message*/)
{
    RCLCPP_INFO(ros_node_->get_logger(), "Stop: zero forces");
    std::lock_guard<std::mutex> lock(cmd_mutex_);
    des_torques_.setZero();
    des_positions_.setZero();
    des_velocities_.setZero();
    des_pos_gains_.setZero();
    des_vel_gains_.setZero();
    return true;
}

}  // namespace odri_ros2_gazebo_plugin

GZ_ADD_PLUGIN(odri_ros2_gazebo_plugin::OdriGazeboPlugin,
              gz::sim::System,
              gz::sim::ISystemConfigure,
              gz::sim::ISystemPreUpdate,
              gz::sim::ISystemPostUpdate)

GZ_ADD_PLUGIN_ALIAS(odri_ros2_gazebo_plugin::OdriGazeboPlugin,
                    "odri_ros2_gazebo_plugin::OdriGazeboPlugin")
