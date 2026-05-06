#include "odri_ros2_hardware/robot_interface.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"

namespace odri_interface
{
RobotInterface::RobotInterface(const std::string& node_name) : Node{node_name}, StateMachineInterface(node_name)
{
    i_gdb = 0;

    declareParameters();

    // Build the robot yaml path
    std::string yaml_path;
    try {
        yaml_path = ament_index_cpp::get_package_share_directory("odri_ros2_hardware");

    } catch (const std::exception& e) {
        // Should never be thrown. It is this same package.
        std::cerr << "Package 'odri_interface' not found: \n" + std::string(e.what()) << '\n';
    }
    yaml_path += "/config/robots/" + params_.robot_yaml_name;

    odri_robot_ = odri_control_interface::RobotFromYamlFile(yaml_path);
    try {
        odri_robot_->Start();
        odri_robot_->WaitUntilReady();
    } catch (const std::exception& e) {
        RCLCPP_FATAL(get_logger(), "Cannot connect to robot: %s", e.what());
        throw;
    }

    pub_joint_states_ = create_publisher<sensor_msgs::msg::JointState>(
        "joint_states", rclcpp::SensorDataQoS());
    pub_robot_state_ = create_publisher<odri_ros2_interfaces::msg::RobotState>("robot_state", rclcpp::SensorDataQoS());
    pub_imu_         = create_publisher<sensor_msgs::msg::Imu>("imu", rclcpp::SensorDataQoS());
    subs_motor_commands_ = create_subscription<odri_ros2_interfaces::msg::RobotCommand>(
        "robot_command", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile(),
        std::bind(&RobotInterface::callbackRobotCommand, this, std::placeholders::_1));

    timer_send_commands_ = create_wall_timer(std::chrono::duration<double, std::milli>(2),
                                             std::bind(&RobotInterface::callbackTimerSendCommands, this));

    positions_  = Eigen::VectorXd::Zero(odri_robot_->joints->GetNumberMotors());
    velocities_ = positions_;
    torques_    = positions_;

    des_torques_    = Eigen::VectorXd::Zero(odri_robot_->joints->GetNumberMotors());
    des_positions_  = Eigen::VectorXd::Zero(odri_robot_->joints->GetNumberMotors());
    des_velocities_ = Eigen::VectorXd::Zero(odri_robot_->joints->GetNumberMotors());
    des_pos_gains_  = Eigen::VectorXd::Zero(odri_robot_->joints->GetNumberMotors());
    des_vel_gains_  = Eigen::VectorXd::Zero(odri_robot_->joints->GetNumberMotors());
    max_currents_   = Eigen::VectorXd::Zero(odri_robot_->joints->GetNumberMotors());

    // Add extra states to default state machine
    state_machine_->addState("calibrating_offsets");

    state_machine_->addTransition("start_calibrating_offsets", "idle", "calibrating_offsets");
    state_machine_->assignTransitionCallback("start_calibrating_offsets",
                                             &RobotInterface::transStartCalibratingOffsetsCallback, this);

    state_machine_->addTransition("end_calibrating_offsets", "calibrating_offsets", "idle");
    state_machine_->assignTransitionCallback("end_calibrating_offsets",
                                             &RobotInterface::transEndCalibratingOffsetsCallback, this);

    state_machine_->addState("calibrating_safe_configuration");

    state_machine_->addTransition("start_calibrating_safe_configuration", "idle", "calibrating_safe_configuration");
    state_machine_->assignTransitionCallback("start_calibrating_safe_configuration",
                                             &RobotInterface::transStartCalibratingSafeConfigurationCallback, this);

    state_machine_->addTransition("end_calibrating_safe_configuration", "calibrating_safe_configuration", "idle");
    state_machine_->assignTransitionCallback("end_calibrating_safe_configuration",
                                             &RobotInterface::transEndCalibratingSafeConfigurationCallback, this);
}

RobotInterface::~RobotInterface() {}

void RobotInterface::declareParameters()
{
    declare_parameter<std::string>("robot_yaml_name", "");
    declare_parameter<int>("n_slaves", 1);

    get_parameter<std::string>("robot_yaml_name", params_.robot_yaml_name);

    int n_slaves;
    get_parameter<int>("n_slaves", n_slaves);
    params_.n_slaves = n_slaves;

    std::vector<double> safe_pos_default(params_.n_slaves * 2, 0.0);
    declare_parameter<std::vector<double>>("safe_configuration", safe_pos_default);
    declare_parameter<double>("safe_torque", 0.0);
    declare_parameter<double>("safe_current", 2.0);
    declare_parameter<double>("safe_kp", 0.5);
    declare_parameter<double>("safe_kd", 0.5);

    std::vector<double> safe_pos;
    get_parameter<std::vector<double>>("safe_configuration", safe_pos);
    params_.safe_configuration = Eigen::Map<Eigen::VectorXd>(safe_pos.data(), params_.n_slaves * 2);
    RCLCPP_INFO_STREAM(get_logger(), "Safe configuration: " << params_.safe_configuration.transpose());
    get_parameter<double>("safe_torque", params_.safe_torque);
    get_parameter<double>("safe_current", params_.safe_current);
    get_parameter<double>("safe_kp", params_.safe_kp);
    get_parameter<double>("safe_kd", params_.safe_kd);

    declare_parameter<std::vector<std::string>>("joint_names", {});
    get_parameter<std::vector<std::string>>("joint_names", params_.joint_names);
}

void RobotInterface::callbackTimerSendCommands()
{
    odri_robot_->ParseSensorData();

    positions_  = odri_robot_->joints->GetPositions();
    velocities_ = odri_robot_->joints->GetVelocities();
    torques_    = odri_robot_->joints->GetMeasuredTorques();

    const rclcpp::Time now = get_clock()->now();

    sensor_msgs::msg::JointState js_msg;
    js_msg.header.stamp = now;
    js_msg.name         = params_.joint_names;
    js_msg.position     = {positions_.data(),  positions_.data()  + positions_.size()};
    js_msg.velocity     = {velocities_.data(), velocities_.data() + velocities_.size()};
    js_msg.effort       = {torques_.data(),    torques_.data()    + torques_.size()};
    pub_joint_states_->publish(js_msg);

    robot_state_msg_.header.stamp = now;
    robot_state_msg_.motor_states.clear();

    if (odri_robot_->IsReady()) {
        // printf("Ready at %ld.\n",i_gdb++);
    }

    for (long int i = 0; i < positions_.size(); ++i) {
        odri_ros2_interfaces::msg::MotorState m_state;

        m_state.position                = positions_(i);
        m_state.velocity                = velocities_(i);
        m_state.torque                  = torques_(i);
        m_state.is_enabled              = odri_robot_->joints->GetEnabled()(i);
        m_state.has_index_been_detected = odri_robot_->joints->HasIndexBeenDetected()(i);

        robot_state_msg_.motor_states.push_back(m_state);
    }
    pub_robot_state_->publish(robot_state_msg_);

    // --- IMU ---
    odri_robot_->imu->ParseSensorData();
    const Eigen::Vector4d& quat  = odri_robot_->imu->GetAttitudeQuaternion();
    const Eigen::Vector3d& gyro  = odri_robot_->imu->GetGyroscope();
    const Eigen::Vector3d& accel = odri_robot_->imu->GetLinearAcceleration();

    imu_msg_.header.stamp    = robot_state_msg_.header.stamp;
    imu_msg_.header.frame_id = "base_link";

    imu_msg_.orientation.x = quat(1);
    imu_msg_.orientation.y = quat(2);
    imu_msg_.orientation.z = quat(3);
    imu_msg_.orientation.w = quat(0);

    imu_msg_.angular_velocity.x    = gyro(0);
    imu_msg_.angular_velocity.y    = gyro(1);
    imu_msg_.angular_velocity.z    = gyro(2);

    imu_msg_.linear_acceleration.x = accel(0);
    imu_msg_.linear_acceleration.y = accel(1);
    imu_msg_.linear_acceleration.z = accel(2);

    pub_imu_->publish(imu_msg_);

    Eigen::VectorXd vec_zero = Eigen::VectorXd::Zero(odri_robot_->GetJoints()->GetNumberMotors());
    if (state_machine_->getStateActive() == "idle" || state_machine_->getStateActive() == "calibrating_offsets" ||
        state_machine_->getStateActive() == "calibrating_safe_configuration") {
        odri_robot_->joints->SetTorques(vec_zero);
        odri_robot_->joints->SetDesiredPositions(vec_zero);
        odri_robot_->joints->SetDesiredVelocities(vec_zero);
        odri_robot_->joints->SetPositionGains(vec_zero);
        odri_robot_->joints->SetVelocityGains(vec_zero);
    } else if (state_machine_->getStateActive() == "enabled") {
        // des_torques_    = params_.safe_configuration.cwiseSign() * params_.safe_torque;
        des_positions_  = params_.safe_configuration;
        des_velocities_ = (params_.safe_configuration - positions_) / 1.0;
        des_pos_gains_.setConstant(params_.safe_kp);
        des_vel_gains_.setConstant(params_.safe_kd);

        // odri_robot_->joints->SetTorques(des_torques_);
        odri_robot_->joints->SetDesiredPositions(des_positions_);
        odri_robot_->joints->SetDesiredVelocities(des_velocities_);
        odri_robot_->joints->SetPositionGains(des_pos_gains_);
        odri_robot_->joints->SetVelocityGains(des_vel_gains_);
        odri_robot_->joints->SetMaximumCurrents(params_.safe_current);

    } else if (state_machine_->getStateActive() == "running") {
        odri_robot_->joints->SetTorques(des_torques_);
        odri_robot_->joints->SetDesiredPositions(des_positions_);
        odri_robot_->joints->SetDesiredVelocities(des_velocities_);
        odri_robot_->joints->SetPositionGains(des_pos_gains_);
        odri_robot_->joints->SetVelocityGains(des_vel_gains_);
        odri_robot_->joints->SetMaximumCurrents(max_currents_(0));  // WARNING: Max current is common for all joints
    }
    odri_robot_->SendCommand();
}

void RobotInterface::callbackRobotCommand(const odri_ros2_interfaces::msg::RobotCommand::SharedPtr msg)
{
    if (state_machine_->getStateActive() == "running") {
        for (std::size_t i = 0; i < msg->motor_commands.size(); ++i) {
            des_torques_(i)    = msg->motor_commands[i].torque_ref;
            des_positions_(i)  = msg->motor_commands[i].position_ref;
            des_velocities_(i) = msg->motor_commands[i].velocity_ref;
            des_pos_gains_(i)  = msg->motor_commands[i].kp;
            des_vel_gains_(i)  = msg->motor_commands[i].kd;
            max_currents_(i)   = msg->motor_commands[i].i_sat;
        }
    }
}

bool RobotInterface::transEnableCallback(std::string& message)
{
    RCLCPP_INFO_STREAM(get_logger(), "\nEnable: Finding indexes and going to zero position");
    Eigen::VectorXd zero_vec = Eigen::VectorXd::Zero(odri_robot_->GetJoints()->GetNumberMotors());
    odri_robot_->RunCalibration(params_.safe_configuration);

    // des_torques_.setZero();
    // des_positions_  = positions_;
    // des_velocities_ = (params_.safe_configuration - positions_) / 2.5;
    // des_pos_gains_.setConstant(5);
    // des_vel_gains_.setConstant(0.5);

    message = "ODRI enabled successful";
    return true;
}

bool RobotInterface::transDisableCallback(std::string& /*message*/) { return true; }

bool RobotInterface::transStartCallback(std::string& /*message*/) { return true; }

bool RobotInterface::transStopCallback(std::string& /*message*/) { return true; }

bool RobotInterface::transStartCalibratingOffsetsCallback(std::string& message)
{
    Eigen::VectorXd zero_vec = Eigen::VectorXd::Zero(odri_robot_->GetJoints()->GetNumberMotors());
    odri_robot_->GetJoints()->SetPositionOffsets(zero_vec);

    RCLCPP_INFO_STREAM(get_logger(), "\n[Calibrating offsets] Finding indexes...");
    odri_robot_->RunCalibration(zero_vec);

    RCLCPP_INFO_STREAM(get_logger(),
                       "\n[Calibrating offsets] follow the steps: "
                       "\n \t1. Put all joints in their respective zero positions. "
                       "\n \t2. Call transition service with 'end_calibration'"
                       "\n \t3. Update the printed offsets in the YAML file");

    message = "Calibrating offsets running. Do required steps.";

    return true;
}

bool RobotInterface::transEndCalibratingOffsetsCallback(std::string& message)
{
    RCLCPP_INFO_STREAM(get_logger(), "\nThese are the offsets. Add them to the YAML file");
    Eigen::VectorXd current_pos = odri_robot_->GetJoints()->GetPositions();

    for (long int i = 0; i < current_pos.size(); i++) {
        RCLCPP_INFO_STREAM(get_logger(), "Joint " << i << ": " << -current_pos(i));
    }

    odri_robot_->GetJoints()->SetPositionOffsets(-current_pos);

    message = "Calibrating offsets done";

    return true;
}

bool RobotInterface::transStartCalibratingSafeConfigurationCallback(std::string& message)
{
    RCLCPP_INFO_STREAM(get_logger(), "\n [Safe configuration calibration] Finding indexes and going to zero position");
    Eigen::VectorXd zero_vec = Eigen::VectorXd::Zero(odri_robot_->GetJoints()->GetNumberMotors());
    odri_robot_->RunCalibration(zero_vec);

    RCLCPP_INFO_STREAM(
        get_logger(),
        "\n[Safe configuration calibration] follow the steps:"
        "\n \t1. Put the joints in the safe configuration"
        "\n \t2. Call transition service with 'end_calibrating_safe_configuration'"
        "\n \t3. Update the field 'safe_configuration' in the yaml file with the printed configuration");

    message = "Calibrating safe configuration running. Do required steps.";

    return true;
}

bool RobotInterface::transEndCalibratingSafeConfigurationCallback(std::string& message)
{
    RCLCPP_INFO_STREAM(get_logger(),
                       "\nThese are the joint values in the safe configuration. Add them to the YAML file");
    Eigen::VectorXd current_pos = odri_robot_->GetJoints()->GetPositions();

    for (long int i = 0; i < current_pos.size(); i++) {
        RCLCPP_INFO_STREAM(get_logger(), "Joint " << i << ": " << current_pos(i));
    }

    message = "Calibrating safe configuration done";

    return true;
}

}  // namespace odri_interface

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    int ret = 0;
    try {
        std::shared_ptr<odri_interface::RobotInterface> master_board_iface =
            std::make_shared<odri_interface::RobotInterface>("RobotInterface");

        rclcpp::executors::SingleThreadedExecutor executor;
        // rclcpp::executors::StaticSingleThreadedExecutor executor;

        executor.add_node(master_board_iface);
        executor.spin();
    } catch (const std::exception&) {
        // already logged with RCLCPP_FATAL in the constructor
        ret = 1;
    }
    rclcpp::shutdown();
    return ret;
}
