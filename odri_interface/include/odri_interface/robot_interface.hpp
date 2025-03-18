#pragma once

#include <chrono>
#include <memory>
#include <vector>

#include <Eigen/Dense>

#include <rclcpp/rclcpp.hpp>

#include "hidro_ros2_utils/state_machine_interface.hpp"

#include "odri_ros2_msgs/msg/driver_state.hpp"
#include "odri_ros2_msgs/msg/master_board_state.hpp"
#include "odri_ros2_msgs/msg/robot_command.hpp"
#include "odri_ros2_msgs/msg/robot_state.hpp"

#include "odri_control_interface/robot.hpp"
#include "odri_control_interface/utils.hpp"

#include "master_board_sdk/master_board_interface.h"
#include "master_board_sdk/defines.h"

namespace odri_interface
{

class RobotInterface : public hidro_ros2_utils::StateMachineInterface
{
    public:
    explicit RobotInterface(const std::string& node_name);
    virtual ~RobotInterface();

    private:
    void declareParameters();

    void callbackTimerSendCommands();
    void callbackRobotCommand(const odri_ros2_msgs::msg::RobotCommand::SharedPtr msg);

    bool smCalibrateFromIdle(std::string& message);
    bool smCalibrateFromCalibrating(std::string& message);

    virtual bool transEnableCallback(std::string& message) override;
    virtual bool transStartCallback(std::string& message) override;
    virtual bool transDisableCallback(std::string& message) override;
    virtual bool transStopCallback(std::string& message) override;

    bool transStartCalibratingOffsetsCallback(std::string& message);
    bool transEndCalibratingOffsetsCallback(std::string& message);

    bool transStartCalibratingSafeConfigurationCallback(std::string& message);
    bool transEndCalibratingSafeConfigurationCallback(std::string& message);

    private:
    rclcpp::TimerBase::SharedPtr timer_send_commands_;

    rclcpp::Publisher<odri_ros2_msgs::msg::RobotState>::SharedPtr      pub_robot_state_;
    rclcpp::Subscription<odri_ros2_msgs::msg::RobotCommand>::SharedPtr subs_motor_commands_;

    std::shared_ptr<odri_control_interface::Robot> odri_robot_;

    odri_ros2_msgs::msg::RobotState robot_state_msg_;

    std::chrono::high_resolution_clock::time_point t_last_mb_command_;

    Eigen::VectorXd positions_;
    Eigen::VectorXd velocities_;
    Eigen::VectorXd torques_;

    Eigen::VectorXd des_torques_;
    Eigen::VectorXd des_positions_;
    Eigen::VectorXd des_velocities_;
    Eigen::VectorXd des_pos_gains_;
    Eigen::VectorXd des_vel_gains_;
    Eigen::VectorXd max_currents_;

    long i_gdb;

    struct Params {
        std::size_t     n_slaves;      // rm (yaml)
        std::string     robot_yaml_name;
        Eigen::VectorXd safe_configuration;
        double          safe_kp;
        double          safe_kd;
        double          safe_torque;
        double          safe_current;
    } params_;
};

}  // namespace odri_interface