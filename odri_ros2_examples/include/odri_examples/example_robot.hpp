#pragma once

#include <Eigen/Dense>

#include <rclcpp/rclcpp.hpp>
// #include "rclcpp_components/register_node_macro.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

#include "hidro_ros2_utils/srv/transition_command.hpp"

#include "odri_ros2_interfaces/msg/robot_state.hpp"
#include "odri_ros2_interfaces/msg/robot_command.hpp"

class ExampleRobot : public rclcpp::Node
{
  public:
    explicit ExampleRobot(const std::string &node_name);
    virtual ~ExampleRobot();

  private:
    void callbackTimerChangeCommand();
    void callbackTimerPublishCommand();
    void callbackRobotState(const odri_ros2_interfaces::msg::RobotState::SharedPtr msg);
    rcl_interfaces::msg::SetParametersResult callbackParameters(const std::vector<rclcpp::Parameter> &parameters);

  private:
    rclcpp::TimerBase::SharedPtr timer_change_command_;
    rclcpp::TimerBase::SharedPtr timer_publish_command_;

    rclcpp::Subscription<odri_ros2_interfaces::msg::RobotState>::SharedPtr sub_robot_state_;
    rclcpp::Publisher<odri_ros2_interfaces::msg::RobotCommand>::SharedPtr  pub_robot_command_;

    rclcpp::Client<hidro_ros2_utils::srv::TransitionCommand>::SharedPtr client_odri_interface_;

    OnSetParametersCallbackHandle::SharedPtr callback_handle_;

    std::chrono::high_resolution_clock::time_point t_last_mb_command_;
    std::chrono::milliseconds                      t_before_zero_commands_;

    odri_ros2_interfaces::msg::RobotCommand msg_robot_command_;

    struct WaveParams
    {
        double amplitude;
        double freq;
        double t;
        double dt;
    } wave_params_;

    bool            brought_to_init_;
    Eigen::Vector2d pos_error_;

    struct Params
    {
        bool publish_commands;
    } params_;

    bool        got_initial_position_;
    std::size_t counter_initial_position_;
};
