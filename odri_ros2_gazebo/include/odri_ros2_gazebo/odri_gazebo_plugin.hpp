#pragma once

#include <Eigen/Dense>

#include <rclcpp/rclcpp.hpp>
#include <gazebo/common/Plugin.hh>

#include "odri_ros2_interfaces/msg/driver_state.hpp"
#include "odri_ros2_interfaces/msg/master_board_state.hpp"
#include "odri_ros2_interfaces/msg/robot_command.hpp"
#include "odri_ros2_interfaces/msg/robot_state.hpp"

#include "hidro_ros2_utils/srv/transition_command.hpp"
#include "hidro_ros2_utils/msg/state_machine_status.hpp"

#include "hidro_utils/state_machine.hpp"

namespace odri_ros2_gazebo_plugin
{

class OdriGazeboPlugin : public gazebo::ModelPlugin
{
  public:
    OdriGazeboPlugin();

    void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf);

  private:
    void initializeRosObjects(sdf::ElementPtr sdf);
    void parseSdf(sdf::ElementPtr sdf);
    void initializeStateMachine();
    void initializeDataObjects();
    void printInfo();

    void Update();
    // ROS
    void callbackRobotCommand(const odri_ros2_interfaces::msg::RobotCommand::SharedPtr msg);

    // StateMachine
    void transitionRequest(const std::shared_ptr<hidro_ros2_utils::srv::TransitionCommand::Request>  request,
                           const std::shared_ptr<hidro_ros2_utils::srv::TransitionCommand::Response> response);

    void timerPublishStateCallback();

    bool transEnableCallback(std::string& message);
    bool transStartCallback(std::string& message);
    bool transDisableCallback(std::string& message);
    bool transStopCallback(std::string& message);

  private:
    std::string robot_namespace_;

    gazebo::physics::ModelPtr model_;
    gazebo::physics::WorldPtr world_;

    gazebo::common::Time last_sim_time_;
    gazebo::common::Time last_update_time_;
    double               update_period_ms_;

    gazebo::event::ConnectionPtr update_connection_;

    std::vector<gazebo::physics::JointPtr> joints_;
    std::vector<std::string>               joint_names_;

    rclcpp::Node::SharedPtr ros_node_;

    // ROS members
    rclcpp::Publisher<odri_ros2_interfaces::msg::RobotState>::SharedPtr      pub_robot_state_;
    rclcpp::Subscription<odri_ros2_interfaces::msg::RobotCommand>::SharedPtr subs_motor_commands_;

    odri_ros2_interfaces::msg::RobotState     robot_state_msg_;
    hidro_ros2_utils::msg::StateMachineStatus state_machine_status_msg_;

    rclcpp::Service<hidro_ros2_utils::srv::TransitionCommand>::SharedPtr    service_sm_transition_;
    rclcpp::Publisher<hidro_ros2_utils::msg::StateMachineStatus>::SharedPtr pub_status_;
    rclcpp::TimerBase::SharedPtr                                            timer_status_pub_;
    std::chrono::duration<double>                                           status_pub_period_;

    // Other members
    hidro_utils::StateMachineDefaultPtr state_machine_;

    Eigen::VectorXd des_torques_;
    Eigen::VectorXd des_positions_;
    Eigen::VectorXd des_velocities_;
    Eigen::VectorXd des_pos_gains_;
    Eigen::VectorXd des_vel_gains_;
    Eigen::VectorXd max_currents_;

    Eigen::VectorXd safe_positions_;
    Eigen::VectorXd safe_torques_;
};

}  // namespace odri_ros2_gazebo_plugin
