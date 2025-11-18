
/**
 * Copyright (C) 2022 ez-Wheel S.A.S.
 *
 * @file DiffDriveController.cpp
 */

#include "diff_drive_controller/DiffDriveController.hpp"

#include <iomanip>
#include <sstream>
#include <string>

#include "ezw-smc-core/Config.hpp"
#include "tf2/LinearMath/Quaternion.h"

#define TIMER_STATE_MACHINE_MS 1000ms
#define TIMER_SAFETY_MS 50ms
#define RETRY_NB 10

// Define threshold for detecting resets
#define RESET_THRESHOLD_M 0.5  // (adjust based on encoder resolution)

using namespace std::chrono_literals;

namespace ezw::swd {
    using std::placeholders::_1;

    DiffDriveController::DiffDriveController(const std::string &p_node_name) : Node(p_node_name)
    {
        // Create parameters
        m_params = std::make_shared<ezw::swd::DiffDriveParameters>(this);

        // Create a TransformBroadcaster object that we'll use later to send transformations
        m_tf2_br = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // Publishers
        if (m_params->getPublishOdom()) {
            m_pub_odom = create_publisher<nav_msgs::msg::Odometry>("odom", 1);
            m_pub_wheel_odometry = create_publisher<swd_ros2_controllers::msg::WheelOdometry>("wheel_odometry", 1);
        }

        if (m_params->getPublishSafety()) {
            m_pub_safety = create_publisher<swd_ros2_controllers::msg::SafetyFunctions>("safety", 1);
        }

        // Subscribers
        m_sub_brake = create_subscription<std_msgs::msg::Bool>("soft_brake", 1, std::bind(&DiffDriveController::cbSoftBrake, this, _1));
        m_sub_command_set_speed = create_subscription<geometry_msgs::msg::Point>("set_speed", 1, std::bind(&DiffDriveController::cbSetSpeed, this, _1));
        m_sub_command_cmd_vel = create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 1, std::bind(&DiffDriveController::cbCmdVel, this, _1));
        m_sub_reset_odometry = create_subscription<std_msgs::msg::Empty>("reset_odometry", 1, std::bind(&DiffDriveController::cbResetOdometry, this, _1));

        // Initialize motors
        ezw_error_t err;

        if (m_params->getRightConfigFile().empty()) {
            RCLCPP_ERROR(get_logger(), "Please specify the 'right_swd_config_file' parameter");
            throw std::runtime_error("Please specify the 'right_swd_config_file' parameter");
        }

        // Connect to the right motor

        /* Load motor config file */
        auto config = std::make_shared<ezw::smccore::Config>();
        err = config->load(m_params->getRightConfigFile());
        if (err != ERROR_NONE) {
            RCLCPP_ERROR(get_logger(),
                         "Failed loading right motor's config file <%s>, CONTEXT_ID: %d, EZW_ERR: SMCService : "
                         "Config.init() return error code : %d",
                         m_params->getRightConfigFile().c_str(), CON_APP, (int)err);
            throw std::runtime_error("Failed loading right motor's config file");
        }

        m_right_wheel_diameter_m = config->getDiameter() * 1e-3;
        m_r_motor_reduction = config->getReduction();

        /* Init DBus client */
        RCLCPP_INFO(get_logger(), "Initializing right dbus client !");

        std::string dbus_namespace = config->getDbusNamespace();
        std::transform(dbus_namespace.begin(), dbus_namespace.end(), dbus_namespace.begin(), ::tolower);

        std::string service_instance_name = "commonapi.ezw.smcservice." + dbus_namespace;

        err = m_right_controller.init(config->getContextId(), "local", service_instance_name);
        if (err != ERROR_NONE) {
            RCLCPP_ERROR(get_logger(),
                         "Failed initializing right motor, EZW_ERR: SMCService : "
                         "Controller::init() return error code : %d",
                         (int)err);
            throw std::runtime_error("Failed initializing right motor");
        }

        if (m_params->getLeftConfigFile().empty()) {
            RCLCPP_ERROR(get_logger(), "Please specify the 'left_swd_config_file' parameter");
            throw std::runtime_error("Please specify the 'left_swd_config_file' parameter");
        }

        /* Load motor config file */
        err = config->load(m_params->getLeftConfigFile());
        if (err != ERROR_NONE) {
            RCLCPP_ERROR(get_logger(),
                         "Failed loading left motor's config file <%s>, CONTEXT_ID: %d, EZW_ERR: SMCService : "
                         "Config.init() return error code : %d",
                         m_params->getLeftConfigFile().c_str(), CON_APP, (int)err);
            throw std::runtime_error("Failed initializing left motor");
        }

        m_left_wheel_diameter_m = config->getDiameter() * 1e-3;
        m_l_motor_reduction = config->getReduction();

        /* Init DBus client */
        RCLCPP_INFO(get_logger(), "Initializing left dbus client !");

        dbus_namespace = config->getDbusNamespace();
        std::transform(dbus_namespace.begin(), dbus_namespace.end(), dbus_namespace.begin(), ::tolower);

        service_instance_name = "commonapi.ezw.smcservice." + dbus_namespace;

        err = m_left_controller.init(config->getContextId(), "local", service_instance_name);
        if (err != ERROR_NONE) {
            RCLCPP_ERROR(get_logger(),
                         "Failed initializing left motor, EZW_ERR: SMCService : "
                         "Controller::init() return error code : %d",
                         (int)err);
            throw std::runtime_error("Failed initializing left motor");
        }

        /* Read initial encoders values */
        ezw_error_t err_l = ezw_error_t::ERROR_INTERNAL, err_r = ezw_error_t::ERROR_INTERNAL;
        int attempt = 0;

        while (err_l != ezw_error_t::ERROR_NONE && attempt++ < RETRY_NB) {
            if (m_params->getAccurateOdometry()) {
                err_l = m_left_controller.getAccurateOdometryValueTS(m_dist_left_prev_mm, m_left_timestamp_prev_us);
            }
            else {
                err_l = m_left_controller.getOdometryValueTS(m_dist_left_prev_mm, m_left_timestamp_prev_us);
            }

            usleep((1000) * 1000);
        }
        if (ERROR_NONE != err_l) {
            RCLCPP_ERROR(get_logger(),
                         "Failed reading from left motor, EZW_ERR: SMCService : "
                         "Controller::%s() return error code : %d",
                         m_params->getAccurateOdometry() ? "getAccurateOdometryValueTS" : "getOdometryValueTS", (int)err_l);
            throw std::runtime_error("Initial reading from left motor failed");
        }

        attempt = 0;
        while (err_r != ezw_error_t::ERROR_NONE && attempt++ < RETRY_NB) {
            if (m_params->getAccurateOdometry()) {
                err_r = m_right_controller.getAccurateOdometryValueTS(m_dist_right_prev_mm, m_right_timestamp_prev_us);
            }
            else {
                err_r = m_right_controller.getOdometryValueTS(m_dist_right_prev_mm, m_right_timestamp_prev_us);
            }

            usleep((1000) * 1000);
        }

        if (ERROR_NONE != err_r) {
            RCLCPP_ERROR(get_logger(),
                         "Failed reading from right motor, EZW_ERR: SMCService : "
                         "Controller::%s() return error code : %d",
                         m_params->getAccurateOdometry() ? "getAccurateOdometryValueTS" : "getOdometryValueTS", (int)err_r);
            throw std::runtime_error("Initial reading from right motor failed");
        }

        // Parameters
        ezw::smccore::IPDSService::PolarityParameters polarity_parameters;
        err = m_left_controller.getPolarityParameters(polarity_parameters);
        if (ERROR_NONE != err) {
            RCLCPP_ERROR(get_logger(),
                         "Failed reading the motor polarity, EZW_ERR: SMCService : "
                         "Controller::getPolarityParameters() return error code : %d",
                         (int)err);
            throw std::runtime_error("Failed reading the left motor polarity");
        }
        m_left_motor_polarity = polarity_parameters.velocity_polarity;

        RCLCPP_INFO(get_logger(), "left motor polarity : %s", m_left_motor_polarity ? "True" : "False");

        ezw::smccore::IVelocityModeService::VelocityModeParameters velocity_mode_parameters;
        err = m_left_controller.getVelocityModeParameters(velocity_mode_parameters);
        if (ERROR_NONE != err) {
            RCLCPP_ERROR(get_logger(),
                         "Failed reading the left motor velocity mode parameters, EZW_ERR: SMCService : "
                         "Controller::getVelocityModeParameters() return error code : %d",
                         (int)err);
            throw std::runtime_error("Failed reading the left motor velocity mode parameters");
        }
        m_left_min_speed_rpm = velocity_mode_parameters.vl_velocity_min_amount;

        RCLCPP_INFO(get_logger(), "min left velocity : %d rpm", m_left_min_speed_rpm);

        err = m_right_controller.getVelocityModeParameters(velocity_mode_parameters);
        if (ERROR_NONE != err) {
            RCLCPP_ERROR(get_logger(),
                         "Failed reading the right motor velocity mode parameters, EZW_ERR: SMCService : "
                         "Controller::getVelocityModeParameters() return error code : %d",
                         (int)err);
            throw std::runtime_error("Failed reading the right motor velocity mode parameters");
        }
        m_right_min_speed_rpm = velocity_mode_parameters.vl_velocity_min_amount;

        RCLCPP_INFO(get_logger(), "min right velocity : %d rpm", m_right_min_speed_rpm);

        // SAFEIN_1 : Mapping Size = 6
        // SafetyFunctionId::STO
        // SafetyFunctionId::SBC_1
        // SafetyFunctionId::SBC_2
        // SafetyFunctionId::SBC_3
        // SafetyFunctionId::SLS_1
        // SafetyFunctionId::SLS_2
        // SafetyFunctionId::SLS_3
        // SafetyFunctionId::SLS_4
        // SafetyFunctionId::SLS_5
        // SafetyFunctionId::SLS_6
        // SafetyFunctionId::SLS_7
        // SafetyFunctionId::SLS_8
        // SafetyFunctionId::SDIP_1
        // SafetyFunctionId::SDIP_2
        // SafetyFunctionId::SDIN_1
        // SafetyFunctionId::SDIN_2
        // SafetyFunctionId::ERROR_ACK
        // SafetyFunctionId::RST_ACK
        ezw::smccore::ISafeMotionService::SafetyWordMapping safety_control_word_mapping;

        err = m_right_controller.getSafetyControlWordMapping(ezw::smccore::ISafeMotionService::SafetyControlWordId::SAFEIN_1, safety_control_word_mapping);
        if (ERROR_NONE != err) {
            RCLCPP_ERROR(get_logger(),
                         "Failed reading right SAFEIN_1 mapping, EZW_ERR: SMCService : "
                         "Controller::getSafetyControlWordMapping() return error code : %d",
                         (int)err);
            throw std::runtime_error("Failed reading the right motor SAFEIN_1 mapping");
        }
        m_right_safety_functions[safety_control_word_mapping.safety_function_0] = 0;
        m_right_safety_functions[safety_control_word_mapping.safety_function_1] = 1;
        m_right_safety_functions[safety_control_word_mapping.safety_function_2] = 2;
        m_right_safety_functions[safety_control_word_mapping.safety_function_3] = 3;
        m_right_safety_functions[safety_control_word_mapping.safety_function_4] = 4;
        m_right_safety_functions[safety_control_word_mapping.safety_function_5] = 5;

        err = m_left_controller.getSafetyControlWordMapping(ezw::smccore::ISafeMotionService::SafetyControlWordId::SAFEIN_1, safety_control_word_mapping);
        if (ERROR_NONE != err) {
            RCLCPP_ERROR(get_logger(),
                         "Failed reading left SAFEIN_1 mapping, EZW_ERR: SMCService : "
                         "Controller::getSafetyControlWordMapping() return error code : %d",
                         (int)err);
            throw std::runtime_error("Failed reading the left motor SAFEIN_1 mapping");
        }
        m_left_safety_functions[safety_control_word_mapping.safety_function_0] = 0;
        m_left_safety_functions[safety_control_word_mapping.safety_function_1] = 1;
        m_left_safety_functions[safety_control_word_mapping.safety_function_2] = 2;
        m_left_safety_functions[safety_control_word_mapping.safety_function_3] = 3;
        m_left_safety_functions[safety_control_word_mapping.safety_function_4] = 4;
        m_left_safety_functions[safety_control_word_mapping.safety_function_5] = 5;

        // Start the timers
        m_timer_watchdog = create_wall_timer(std::chrono::milliseconds(m_params->getWatchdogReceiveMs()), std::bind(&DiffDriveController::cbTimerWatchdogReceive, this));
        m_timer_pds = create_wall_timer(TIMER_STATE_MACHINE_MS, std::bind(&DiffDriveController::cbTimerStateMachine, this));

        if (m_params->getPublishOdom() || m_params->getPublishTf()) {
            m_timer_odom = create_wall_timer(std::chrono::milliseconds(1000 / m_params->getPubFreqHz()), std::bind(&DiffDriveController::cbTimerOdom, this));
        }

        m_timer_safety = create_wall_timer(TIMER_SAFETY_MS, std::bind(&DiffDriveController::cbTimerSafety, this));

        RCLCPP_INFO(get_logger(), "swd_diff_drive_controller initialized successfully!");
    }

    DiffDriveController::~DiffDriveController()
    {
        RCLCPP_INFO(get_logger(), "Set the robot velocity to zero");

        // Stop the robot
        setSpeeds(0, 0);
    }

    void DiffDriveController::cbTimerStateMachine()
    {
        static bool m_first_entry = true;
        ezw_error_t err_l, err_r;

        // PDOs initialization
        bool lIsConnected = false;
        bool lIsPdoInitSucceeded = false;
        err_l = m_left_controller.getConnectionStatus(lIsConnected, lIsPdoInitSucceeded);
        if (ERROR_NONE != err_l) {
            RCLCPP_ERROR(get_logger(),
                         "Failed to get connection status, EZW_ERR: SMCService : "
                         "Controller.getConnectionStatus() return error code : %d",
                         (int)err_l);
        }

        if (lIsPdoInitSucceeded == false) {
            RCLCPP_WARN(get_logger(),
                        "Left motor PDOs are not initialized, trying to initialize them again");
            err_l = m_left_controller.initializePdo();
            if (ERROR_NONE != err_l) {
                RCLCPP_ERROR(get_logger(),
                             "Failed to initialize left motor PDOs, EZW_ERR: SMCService : "
                             "Controller.initializePdo() return error code : %d",
                             (int)err_l);
            }
        }

        err_r = m_right_controller.getConnectionStatus(lIsConnected, lIsPdoInitSucceeded);
        if (ERROR_NONE != err_r) {
            RCLCPP_ERROR(get_logger(),
                         "Failed to get connection status, EZW_ERR: SMCService : "
                         "Controller.getConnectionStatus() return error code : %d",
                         (int)err_r);
        }
        if (lIsPdoInitSucceeded == false) {
            RCLCPP_WARN(get_logger(),
                        "Right motor PDOs are not initialized, trying to initialize them again");
            err_r = m_right_controller.initializePdo();
            if (ERROR_NONE != err_r) {
                RCLCPP_ERROR(get_logger(),
                             "Failed to initialize right motor PDOs, EZW_ERR: SMCService : "
                             "Controller.initializePdo() return error code : %d",
                             (int)err_r);
            }
        }

        // NMT state machine
        smccore::INMTService::NMTState nmt_state_l, nmt_state_r;
        smccore::IPDSService::PDSState pds_state_l, pds_state_r;

        pds_state_l = pds_state_r = smccore::IPDSService::PDSState::SWITCH_ON_DISABLED;

        err_l = m_left_controller.getNMTState(nmt_state_l);
        if (ERROR_NONE != err_l) {
            RCLCPP_ERROR(get_logger(),
                         "Failed to get the NMT state for left motor, EZW_ERR: SMCService : "
                         "Controller::getNMTState() return error code : %d",
                         (int)err_l);
        }

        err_r = m_right_controller.getNMTState(nmt_state_r);
        if (ERROR_NONE != err_r) {
            RCLCPP_ERROR(get_logger(),
                         "Failed to get the NMT state for right motor, EZW_ERR: SMCService : "
                         "Controller::getNMTState() return error code : %d",
                         (int)err_r);
        }

        bool nmt_ok = (smccore::INMTService::NMTState::OPER == nmt_state_l) && (smccore::INMTService::NMTState::OPER == nmt_state_r);

        if (m_first_entry || m_nmt_ok != nmt_ok) {
            RCLCPP_INFO(get_logger(), "NMT state machine is %s.", nmt_ok ? "OK" : "not OK");
            m_nmt_ok = nmt_ok;
        }

        if (!m_nmt_ok) {
            // Broadcast NMT command PREOP to all canopen nodes
            ezw_error_t err = m_left_controller.broadcastNMTState(smccore::INMTService::NMTCommand::PREOP);
            if (ERROR_NONE != err) {
                RCLCPP_ERROR(get_logger(),
                             "Failed to broadcast NMT command PREOP"
                             "Controller::broadcastNMTState() return error code : %d",
                             (int)err);
            }
            else {
                usleep((10) * 1000);
                // Broadcast NMT command OPER to all canopen nodes
                err = m_left_controller.broadcastNMTState(smccore::INMTService::NMTCommand::OPER);
                if (ERROR_NONE != err) {
                    RCLCPP_ERROR(get_logger(),
                                 "Failed to broadcast NMT state OPER"
                                 "Controller::broadcastNMTState() return error code : %d",
                                 (int)err);
                }
            }
        }

        bool pds_ok = false;

        // If NMT is operational, check the PDS state
        if (m_nmt_ok) {
            // PDS state machine
            err_l = m_left_controller.getPDSState(pds_state_l);
            err_r = m_right_controller.getPDSState(pds_state_r);

            if (ERROR_NONE != err_l) {
                RCLCPP_ERROR(get_logger(),
                             "Failed to get the PDS state for left motor, EZW_ERR: SMCService : "
                             "Controller::getPDSState() return error code : %d",
                             (int)err_l);
            }

            if (ERROR_NONE != err_r) {
                RCLCPP_ERROR(get_logger(),
                             "Failed to get the PDS state for right motor, EZW_ERR: SMCService : "
                             "Controller::getPDSState() return error code : %d",
                             (int)err_r);
            }

            pds_ok = (smccore::IPDSService::PDSState::OPERATION_ENABLED == pds_state_l) && (smccore::IPDSService::PDSState::OPERATION_ENABLED == pds_state_r);

            if (!pds_ok) {
                // Reading STO
                m_safety_msg_mtx.lock();
                bool sto_signal = m_safety_msg.safe_torque_off;
                m_safety_msg_mtx.unlock();

                if (!sto_signal && smccore::IPDSService::PDSState::OPERATION_ENABLED != pds_state_l) {
                    err_l = m_left_controller.enterInOperationEnabledState();
                }

                if (!sto_signal && smccore::IPDSService::PDSState::OPERATION_ENABLED != pds_state_r) {
                    err_r = m_right_controller.enterInOperationEnabledState();
                }
            }
        }

        if (m_first_entry || m_pds_ok != pds_ok) {
            RCLCPP_INFO(get_logger(), "PDS state machine is %s.", pds_ok ? "OK" : "not OK");
            m_pds_ok = pds_ok;
        }

        m_first_entry = false;
    }

    void DiffDriveController::cbSoftBrake(const std_msgs::msg::Bool::SharedPtr p_msg)
    {
        // true => Enable brake
        // false => Release brake
        ezw_error_t err = m_left_controller.setHalt(p_msg->data != 0);
        if (ERROR_NONE != err) {
            RCLCPP_ERROR(get_logger(), "SoftBrake: Failed %s left motor, EZW_ERR: %d", p_msg->data ? "braking" : "releasing", (int)err);
        }
        else {
            RCLCPP_INFO(get_logger(), "SoftBrake: Left motor's soft brake %s", p_msg->data ? "activated" : "disabled");
        }

        err = m_right_controller.setHalt(p_msg->data != 0);
        if (ERROR_NONE != err) {
            RCLCPP_ERROR(get_logger(), "SoftBrake: Failed %s right motor, EZW_ERR: %d", p_msg->data ? "braking" : "releasing", (int)err);
        }
        else {
            RCLCPP_INFO(get_logger(), "SoftBrake: Right motor's soft brake %s", p_msg->data ? "activated" : "disabled");
        }
    }

    void DiffDriveController::cbTimerOdom()
    {
        nav_msgs::msg::Odometry msg_odom;

        int32_t left_dist_now_mm = 0, right_dist_now_mm = 0;
        uint64_t left_timestamp_us = 0, right_timestamp_us = 0;

        ezw_error_t err_l, err_r;

        if (m_params->getAccurateOdometry()) {
            err_l = m_left_controller.getAccurateOdometryValueTS(left_dist_now_mm, left_timestamp_us);
            err_r = m_right_controller.getAccurateOdometryValueTS(right_dist_now_mm, right_timestamp_us);
        }
        else {
            err_l = m_left_controller.getOdometryValueTS(left_dist_now_mm, left_timestamp_us);
            err_r = m_right_controller.getOdometryValueTS(right_dist_now_mm, right_timestamp_us);
        }

        if (ERROR_NONE != err_l) {
            RCLCPP_ERROR(get_logger(),
                         "Failed reading from left motor, EZW_ERR: SMCService : "
                         "Controller::%s() return error code : %d",
                         m_params->getAccurateOdometry() ? "getAccurateOdometryValueTS" : "getOdometryValueTS", (int)err_l);
            return;
        }

        if (ERROR_NONE != err_r) {
            RCLCPP_ERROR(get_logger(),
                         "Failed reading from right motor, EZW_ERR: SMCService : "
                         "Controller::%s() return error code : %d",
                         m_params->getAccurateOdometry() ? "getAccurateOdometryValueTS" : "getOdometryValueTS", (int)err_r);
            return;
        }

        if (m_left_timestamp_prev_us == left_timestamp_us || m_right_timestamp_prev_us == right_timestamp_us) {
            // Nothing to do
            // Values have not changed (avoid "nan" values in /odom topic)
            return;
        }

        // Acquire odom mutex
        std::lock_guard<std::mutex> lock(m_odom_msg_mtx);

        // Encoder difference between t and t-1
        double d_dist_left_m = static_cast<double>(left_dist_now_mm - m_dist_left_prev_mm) / 1000.0;
        double d_dist_right_m = static_cast<double>(right_dist_now_mm - m_dist_right_prev_mm) / 1000.0;

        // Detect left encoder reset
        if (std::abs(d_dist_left_m) > RESET_THRESHOLD_M) {
            d_dist_left_m = 0.0;                              // Fix distance to avoid jumps
            m_left_encoder_offset_mm += m_dist_left_prev_mm;  // Inc offset to be continuous after reset
            RCLCPP_WARN(get_logger(), "Left encoder reset detected!");
        }

        // Detect right encoder reset
        if (std::abs(d_dist_right_m) > RESET_THRESHOLD_M) {
            d_dist_right_m = 0.0;                               // Fix distance to avoid jumps
            m_right_encoder_offset_mm += m_dist_right_prev_mm;  // Inc offset to be continuous after reset
            RCLCPP_WARN(get_logger(), "Right encoder reset detected!");
        }

        // Time difference between t and t-1
        auto dt_s = 1.0 / m_params->getPubFreqHz();
        auto left_dt_s = (left_timestamp_us - m_left_timestamp_prev_us) / 1000000.0;
        auto right_dt_s = (right_timestamp_us - m_right_timestamp_prev_us) / 1000000.0;

        // Encoder difference normalization
        auto d_dist_left_norm_m = d_dist_left_m * dt_s / left_dt_s;
        auto d_dist_right_norm_m = d_dist_right_m * dt_s / right_dt_s;

        // Kinematic model
        double d_dist_center = (d_dist_left_norm_m + d_dist_right_norm_m) / 2.0;
        double d_theta = (d_dist_right_norm_m - d_dist_left_norm_m) / m_params->getBaseline();

        // Error calculation (standard deviation)
        double d_dist_left_err_m = m_params->getLeftEncoderRelativeError() * std::abs(d_dist_left_norm_m);
        double d_dist_right_err_m = m_params->getRightEncoderRelativeError() * std::abs(d_dist_right_norm_m);

        // Error propagation (See https://en.wikipedia.org/wiki/Propagation_of_uncertainty#Non-linear_combinations)
        double d_dist_center_err = std::sqrt(std::pow(d_dist_left_err_m / 2.0, 2) + std::pow(d_dist_right_err_m / 2.0, 2));
        double d_theta_err = std::sqrt(std::pow(d_dist_left_err_m / m_params->getBaseline(), 2) + std::pow(d_dist_right_err_m / m_params->getBaseline(), 2));

        // Odometry model, integration of the diff drive kinematic model
        double x_now = m_x_prev + d_dist_center * std::cos(m_theta_prev);
        double y_now = m_y_prev + d_dist_center * std::sin(m_theta_prev);
        double theta_now = M_BOUND_ANGLE(m_theta_prev + d_theta);

        // Error propagation
        double x_now_err = std::sqrt(std::pow(m_x_prev_err, 2) + std::pow(std::cos(m_theta_prev) * d_dist_center_err, 2) + std::pow(-std::sin(m_theta_prev) * d_dist_center * m_theta_prev_err, 2));
        double y_now_err = std::sqrt(std::pow(m_y_prev_err, 2) + std::pow(std::sin(m_theta_prev) * d_dist_center_err, 2) + std::pow(std::cos(m_theta_prev) * d_dist_center * m_theta_prev_err, 2));
        double theta_now_err = std::sqrt(std::pow(m_theta_prev_err, 2) + std::pow(d_theta_err, 2));

        auto timestamp = get_clock()->now();

        msg_odom.header.stamp = timestamp;
        msg_odom.header.frame_id = m_params->getOdomFrame();
        msg_odom.child_frame_id = m_params->getBaseFrame();
        msg_odom.twist = geometry_msgs::msg::TwistWithCovariance();
        msg_odom.twist.twist.linear.x = d_dist_center / dt_s;
        msg_odom.twist.twist.angular.z = d_theta / dt_s;
        // RCLCPP_INFO(get_logger(), ";%f;%f;%f;%f;%f;%d;%d", dt_s, d_dist_center, d_theta, msg_odom.twist.twist.linear.x, msg_odom.twist.twist.angular.z, left_dist_now_mm - m_dist_left_prev_mm, right_dist_now_mm - m_dist_right_prev_mm);

        // Set uncertainties for linear and angular velocities (6 * 6) matrix (x y z Rx Ry Rz)
        msg_odom.twist.covariance[0] = std::pow(d_dist_center_err / dt_s, 2);
        msg_odom.twist.covariance[35] = std::pow(d_theta_err / dt_s, 2);

        msg_odom.pose.pose.position.x = x_now;
        msg_odom.pose.pose.position.y = y_now;
        msg_odom.pose.pose.position.z = 0.0;

        tf2::Quaternion quat_orientation;
        quat_orientation.setRPY(0.0, 0.0, theta_now);
        msg_odom.pose.pose.orientation.x = quat_orientation.getX();
        msg_odom.pose.pose.orientation.y = quat_orientation.getY();
        msg_odom.pose.pose.orientation.z = quat_orientation.getZ();
        msg_odom.pose.pose.orientation.w = quat_orientation.getW();

        // Set uncertainties for x, y, and theta (Rz)
        msg_odom.pose.covariance[0] = std::pow(x_now_err, 2);
        msg_odom.pose.covariance[7] = std::pow(y_now_err, 2);
        msg_odom.pose.covariance[35] = std::pow(theta_now_err, 2);

        if (m_params->getPublishOdom()) {
            m_pub_odom->publish(msg_odom);
        }

        if (m_params->getPublishWheelOdometry()) {
            auto msg_wheel_odometry = swd_ros2_controllers::msg::WheelOdometry();

            msg_wheel_odometry.header.stamp = timestamp;
            msg_wheel_odometry.left_wheel_position = left_dist_now_mm + m_left_encoder_offset_mm;
            msg_wheel_odometry.right_wheel_position = right_dist_now_mm + m_right_encoder_offset_mm;
            msg_wheel_odometry.baseline = m_params->getBaseline();

            m_pub_wheel_odometry->publish(msg_wheel_odometry);
        }

        if (m_params->getPublishTf()) {
            geometry_msgs::msg::TransformStamped tf_odom_baselink;
            tf_odom_baselink.header.stamp = timestamp;
            tf_odom_baselink.header.frame_id = m_params->getOdomFrame();
            tf_odom_baselink.child_frame_id = m_params->getBaseFrame();

            tf_odom_baselink.transform.translation.x = msg_odom.pose.pose.position.x;
            tf_odom_baselink.transform.translation.y = msg_odom.pose.pose.position.y;
            tf_odom_baselink.transform.translation.z = msg_odom.pose.pose.position.z;
            tf_odom_baselink.transform.rotation.x = msg_odom.pose.pose.orientation.x;
            tf_odom_baselink.transform.rotation.y = msg_odom.pose.pose.orientation.y;
            tf_odom_baselink.transform.rotation.z = msg_odom.pose.pose.orientation.z;
            tf_odom_baselink.transform.rotation.w = msg_odom.pose.pose.orientation.w;

            // Send TF
            m_tf2_br->sendTransform(tf_odom_baselink);
        }

        m_x_prev = x_now;
        m_y_prev = y_now;
        m_theta_prev = theta_now;
        m_x_prev_err = x_now_err;
        m_y_prev_err = y_now_err;
        m_theta_prev_err = theta_now_err;
        m_dist_left_prev_mm = left_dist_now_mm;
        m_dist_right_prev_mm = right_dist_now_mm;
        m_left_timestamp_prev_us = left_timestamp_us;
        m_right_timestamp_prev_us = right_timestamp_us;
    }

    void DiffDriveController::cbSetSpeed(const geometry_msgs::msg::Point::SharedPtr p_speed)
    {
        m_timer_watchdog->reset();

        // Convert rad/s wheel speed to rpm motor speed
        auto left = static_cast<int32_t>(p_speed->x * m_l_motor_reduction * 60.0 / (2.0 * M_PI));
        auto right = static_cast<int32_t>(p_speed->y * m_r_motor_reduction * 60.0 / (2.0 * M_PI));

#if VERBOSE_OUTPUT
        RCLCPP_INFO(get_logger(),
                    "Got RightLeftSpeeds command: (left, right) = (%f, %f) rad/s. "
                    "Calculated speeds (left, right) = (%d, %d) rpm",
                    p_speed->x, p_speed->y, left, right);
#endif

        setSpeeds(left, right);
    }

    void DiffDriveController::cbCmdVel(const geometry_msgs::msg::Twist::SharedPtr p_cmd_vel)
    {
        m_timer_watchdog->reset();

        double left_vel, right_vel;

        // Control model (diff drive)
        // Target wheel speed (rad/s) from linear command
        double left_vel_from_linear = (2.0 * p_cmd_vel->linear.x) / m_left_wheel_diameter_m;
        double right_vel_from_linear = (2.0 * p_cmd_vel->linear.x) / m_right_wheel_diameter_m;

        // Target wheel speed (rad/s) from angular command
        double left_vel_from_angular = -(p_cmd_vel->angular.z * m_params->getBaseline()) / m_left_wheel_diameter_m;
        double right_vel_from_angular = (p_cmd_vel->angular.z * m_params->getBaseline()) / m_right_wheel_diameter_m;

        // Max wheel speed (rad/s)
        double left_max_rad_s = (m_params->getMotorMaxSpeedRpm() * 2.0 * M_PI) / (60.0 * m_l_motor_reduction);
        double right_max_rad_s = (m_params->getMotorMaxSpeedRpm() * 2.0 * M_PI) / (60.0 * m_r_motor_reduction);

        // Limit angular speed to max motor speed
        if (left_vel_from_angular > left_max_rad_s) {
            RCLCPP_INFO(get_logger(), "Limit left angular speed from %f to %f rad/s", left_vel_from_angular, left_max_rad_s);
            left_vel_from_angular = left_max_rad_s;
        }
        else if (left_vel_from_angular < -left_max_rad_s) {
            RCLCPP_INFO(get_logger(), "Limit left angular speed from %f to %f rad/s", left_vel_from_angular, -left_max_rad_s);
            left_vel_from_angular = -left_max_rad_s;
        }

        if (right_vel_from_angular > right_max_rad_s) {
            RCLCPP_INFO(get_logger(), "Limit right angular speed from %f to %f rad/s", right_vel_from_angular, right_max_rad_s);
            right_vel_from_angular = right_max_rad_s;
        }
        else if (right_vel_from_angular < -right_max_rad_s) {
            RCLCPP_INFO(get_logger(), "Limit right angular speed from %f to %f rad/s", right_vel_from_angular, -right_max_rad_s);
            right_vel_from_angular = -right_max_rad_s;
        }

        // Compute wheel speed
        left_vel = left_vel_from_linear + left_vel_from_angular;
        right_vel = right_vel_from_linear + right_vel_from_angular;

        // Check motor saturation (linear + angular > max)
        double saturation_left = 0.0;
        double saturation_right = 0.0;

        if (left_vel > left_max_rad_s)
            saturation_left = left_vel - left_max_rad_s;  // ex 500
        else if (left_vel < -left_max_rad_s)
            saturation_left = left_vel + left_max_rad_s;  // ex -500

        if (right_vel > right_max_rad_s)
            saturation_right = right_vel - right_max_rad_s;
        else if (right_vel < -right_max_rad_s)
            saturation_right = right_vel + right_max_rad_s;

        // Compute saturation
        double saturation = 0.0;

        if (abs(saturation_left) > abs(saturation_right))
            saturation = saturation_left;
        else
            saturation = saturation_right;

        auto lv = left_vel;
        auto rv = right_vel;

        // Apply saturation: prioritize angular over linear
        if (left_vel > 0.0)
            left_vel -= abs(saturation);
        else
            left_vel += abs(saturation);

        if (right_vel > 0.0)
            right_vel -= abs(saturation);
        else
            right_vel += abs(saturation);

        if (abs(saturation_left) > 0.001) {
            RCLCPP_DEBUG(get_logger(), "Left speed saturation : %f rad/s", saturation_left);
        }

        if (abs(saturation_right) > 0.001) {
            RCLCPP_DEBUG(get_logger(), "Right speed saturation : %f rad/s", saturation_right);
        }

        // left_vel = (2. * p_cmd_vel->linear.x - p_cmd_vel->angular.z * m_baseline_m) / m_left_wheel_diameter_m;
        // right_vel = (2. * p_cmd_vel->linear.x + p_cmd_vel->angular.z * m_baseline_m) / m_right_wheel_diameter_m;

        // Convert rad/s wheel speed to rpm motor speed
        auto left = static_cast<int32_t>(left_vel * m_l_motor_reduction * 60.0 / (2.0 * M_PI));
        auto right = static_cast<int32_t>(right_vel * m_r_motor_reduction * 60.0 / (2.0 * M_PI));

        if (left_vel < lv) {
            RCLCPP_DEBUG(get_logger(), "Left speed desired:%f max:%f vel_rad_s:%f vel_rpm:%d", left_vel, left_max_rad_s, lv, left);
        }
        if (right_vel < rv) {
            RCLCPP_DEBUG(get_logger(), "Right speed desired:%f max:%f vel_rad_s:%f vel_rpm:%d", right_vel, right_max_rad_s, rv, right);
        }

        // // Control model (diff drive)
        // left_vel = (2. * p_cmd_vel->linear.x - p_cmd_vel->angular.z * m_params->getBaseline()) / m_left_wheel_diameter_m;
        // right_vel = (2. * p_cmd_vel->linear.x + p_cmd_vel->angular.z * m_params->getBaseline()) / m_right_wheel_diameter_m;

        // // Convert rad/s wheel speed to rpm motor speed
        // auto left = static_cast<int32_t>(left_vel * m_l_motor_reduction * 60.0 / (2.0 * M_PI));
        // auto right = static_cast<int32_t>(right_vel * m_r_motor_reduction * 60.0 / (2.0 * M_PI));

#if VERBOSE_OUTPUT
        RCLCPP_INFO(get_logger(),
                    "Got Twist command: linear = %f m/s, angular = %f rad/s. "
                    "Calculated speeds (left, right) = (%d, %d) rpm",
                    p_cmd_vel->linear.x, p_cmd_vel->angular.z, left, right);
#endif

        setSpeeds(left, right);

#if VERBOSE_OUTPUT
        auto left_requested = left;
        auto right_requested = right;

        int32_t left_speed, right_speed;

        ezw_error_t err = m_left_controller.getVelocityActualValue(left_speed);
        if (ERROR_NONE != err) {
            RCLCPP_ERROR(get_logger(),
                         "Failed get velocity of left motor, EZW_ERR: SMCService : "
                         "Controller::getTargetVelocity() return error code : %d",
                         (int)err);
            return;
        }

        err = m_right_controller.getVelocityActualValue(right_speed);
        if (ERROR_NONE != err) {
            RCLCPP_ERROR(get_logger(),
                         "Failed get velocity of right motor, EZW_ERR: SMCService : "
                         "Controller::getTargetVelocity() return error code : %d",
                         (int)err);
            return;
        }

        double _left_vel = 1.0 * (left_speed / m_l_motor_reduction / 60.0 * (2.0 * M_PI));
        double _right_vel = 1.0 * (right_speed / m_l_motor_reduction / 60.0 * (2.0 * M_PI));

        double x = 0.5 * ((_left_vel + _right_vel) * m_left_wheel_diameter_m / (4 * m_params->getBaseline()));
        double z = 1.0 * ((_right_vel - _left_vel) * m_left_wheel_diameter_m / (2 * m_params->getBaseline()));

        RCLCPP_INFO(get_logger(),
                    "Twist command (linear.x_requested, angular.z_requested, linear.x_real, angular.z_real); %f;%f;%f;%f; \
                    Calculated speeds (left_requested, right_requested, left_send, right_send); \
                    % d; % d; % d; % d; \
                    Safety indicators (STO, SDIp, SLS_1, SLS_2); % d; % d; % d; % d; ",
                    p_cmd_vel->linear.x,
                    p_cmd_vel->angular.z, x, z, left_requested, right_requested, left, right,
                    (int)m_safety_msg.safe_torque_off, (int)m_safety_msg.safe_direction_indication_forward, (int)m_safety_msg.safety_limited_speed_1, (int)m_safety_msg.safety_limited_speed_2);

#endif
    }

#define CONF_MAX_DELTA_SPEED_SLS_1 (m_params->getMotorMaxSls1SpeedRpm() / 2)  // in rpm motor
#define CONF_MAX_DELTA_SPEED_SLS_2 (m_params->getMotorMaxSls2SpeedRpm() / 2)  // in rpm motor
#define CONF_MAX_DELTA_SPEED (m_params->getMotorMaxDeltaSpeedRpm())           // in rpm motor

    void DiffDriveController::setSpeeds(int32_t p_left_speed, int32_t p_right_speed)
    {
        ezw_error_t err = ERROR_NONE;

        // Get the outer motor speed
        int32_t faster_motor_speed = M_MAX(std::abs(p_left_speed), std::abs(p_right_speed));
        int32_t speed_limit = -1;
        bool max_limited = false;
        bool sls_limited = false;
        int8_t enabled_sls_num = -1;

        // Limit to the maximum allowed speed
        if (faster_motor_speed > m_params->getMotorMaxSpeedRpm()) {
            speed_limit = m_params->getMotorMaxSpeedRpm();
            max_limited = true;
        }

        m_safety_msg_mtx.lock();
        // Reading SLS_1/SLS_2
        bool sls_1_signal = m_safety_msg.safety_limited_speed_1;
        bool sls_2_signal = m_safety_msg.safety_limited_speed_2;
        // Reading SDIp/SDIn
        bool sdi_p_signal = m_safety_msg.safe_direction_indication_forward;
        bool sdi_n_signal = m_safety_msg.safe_direction_indication_backward;
        m_safety_msg_mtx.unlock();

        // If SLS detected, impose the safety limited speed (SLS)
        if (sls_1_signal && (faster_motor_speed > m_params->getMotorMaxSls1SpeedRpm())) {
            speed_limit = m_params->getMotorMaxSls1SpeedRpm();
            sls_limited = true;
            enabled_sls_num = 1;
        }
        else if (sls_2_signal && (faster_motor_speed > m_params->getMotorMaxSls2SpeedRpm())) {
            speed_limit = m_params->getMotorMaxSls2SpeedRpm();
            sls_limited = true;
            enabled_sls_num = 2;
        }

        // Impose the safety limited speed (SLS) in backward movement when the robot doesn't have backward SLS signal.
        // For example, if it has only one forward-facing safety LiDAR, when the robot move backwards, there's no
        // safety guarantees, hence speed is limited to SLS, otherwise, the safety limit will be decided by the
        // presence of the SLS signal.
        if (!m_params->getHaveBackwardSls() && (p_left_speed < 0) && (p_right_speed < 0) && (faster_motor_speed > m_params->getMotorMaxSls1SpeedRpm())) {
            speed_limit = m_params->getMotorMaxSls1SpeedRpm();
        }

        // The left and right motors may have different speeds.
        // If we need to limit one of them, we need to scale the second motor speed.
        // This ensures a speed limitation without distorting the target path.
        if (-1 != speed_limit) {
            // If we enter here, we are sure that (faster_motor_speed > speed_limit).
            // Get the ratio between the outer (faster) motor, and the speed limit.
            double speed_ratio = static_cast<double>(speed_limit) / static_cast<double>(faster_motor_speed);

            // Scale right speed
            p_right_speed = static_cast<int32_t>(static_cast<double>(p_right_speed) * speed_ratio);

            // Scale left speed
            p_left_speed = static_cast<int32_t>(static_cast<double>(p_left_speed) * speed_ratio);

            if (max_limited && sls_limited) {
                RCLCPP_DEBUG(get_logger(),
                             "The target speed exceeds the MAX/SLS_%d maximum speed limit (%d rpm). "
                             "Set speed to (left, right) (%d, %d) rpm",
                             enabled_sls_num, speed_limit, p_left_speed, p_right_speed);
            }
            else if (sls_limited) {
                RCLCPP_DEBUG(get_logger(),
                             "The target speed exceeds the SLS_%d maximum speed limit (%d rpm). "
                             "Set speed to (left, right) (%d, %d) rpm",
                             enabled_sls_num, speed_limit, p_left_speed, p_right_speed);
            }
            else if (max_limited) {
                RCLCPP_DEBUG(get_logger(),
                             "The target speed exceeds the maximum speed limit (%d rpm). "
                             "Set speed to (left, right) (%d, %d) rpm",
                             speed_limit, p_left_speed, p_right_speed);
            }
        }

        // Get the delta wheel speed
        int32_t delta_wheel_speed = std::abs(p_left_speed - p_right_speed);
        int32_t delta_speed_limit = -1;

        // Limit to the maximum allowed delta speed
        if (delta_wheel_speed > CONF_MAX_DELTA_SPEED) {
            delta_speed_limit = CONF_MAX_DELTA_SPEED;
        }

        // If SLS detected, limit to the maximum allowed delta safety limited speed (SLS)
        if (sls_1_signal && (delta_wheel_speed > CONF_MAX_DELTA_SPEED_SLS_1)) {
            delta_speed_limit = CONF_MAX_DELTA_SPEED_SLS_1;
        }
        else if (sls_2_signal && (delta_wheel_speed > CONF_MAX_DELTA_SPEED_SLS_2)) {
            delta_speed_limit = CONF_MAX_DELTA_SPEED_SLS_2;
        }

        // The left and right wheels may have different speeds.
        // If we need to limit one of them, we need to scale the second wheel speed.
        // This ensures a delta speed limitation without distorting the target path.
        if (-1 != delta_speed_limit) {
            // Get the ratio between the max allowed delta speed limit, and the current delta speed limit.
            double delta_speed_ratio = static_cast<double>(delta_speed_limit) / static_cast<double>(delta_wheel_speed);

            // Scale right speed
            p_right_speed = static_cast<int32_t>(static_cast<double>(p_right_speed) * delta_speed_ratio);

            // Scale left speed
            p_left_speed = static_cast<int32_t>(static_cast<double>(p_left_speed) * delta_speed_ratio);

            RCLCPP_DEBUG(get_logger(),
                         "The target speed exceeds the maximum delta speed limit (%d rpm). "
                         "Speed set to (left, right) (%d, %d) rpm",
                         delta_speed_limit, p_left_speed, p_right_speed);
        }

        // If left minimum speed detected, impose the minimum speed
        bool left_min_limit = std::abs(p_left_speed) > 1 && std::abs(p_left_speed) <= m_left_min_speed_rpm;

        // If right minimum speed detected, impose the minimum speed
        bool right_min_limit = std::abs(p_right_speed) > 1 && std::abs(p_right_speed) <= m_right_min_speed_rpm;

        if (left_min_limit || right_min_limit) {
            int32_t left_speed = p_left_speed;
            int32_t right_speed = p_right_speed;

            // Update left speed
            if (left_min_limit) {
                p_left_speed = (p_left_speed > 0) ? m_left_min_speed_rpm : -m_left_min_speed_rpm;
            }

            // Update right speed
            if (right_min_limit) {
                p_right_speed = (p_right_speed > 0) ? m_right_min_speed_rpm : -m_right_min_speed_rpm;
            }

            RCLCPP_DEBUG(get_logger(),
                         "The target speed falls behind the minimum speed limit (left, right) (%d, %d rpm)."
                         "Set speed to (left, right) (%d, %d) rpm",
                         left_speed, right_speed, p_left_speed, p_right_speed);
        }

        // Is SDIp signal detected during turn in place?
        auto sdi_during_turn_in_place = false;
        if (m_params->getCheckSDIDuringTurnInPlace() && (sdi_p_signal || sdi_n_signal) && (p_left_speed * p_right_speed < 0)) {
            p_left_speed = p_right_speed = 0;
            RCLCPP_DEBUG(get_logger(), "SDI signal detected during turn in place. Setting wheel speeds to (left, right) = (%d, %d) rpm", p_left_speed, p_right_speed);
            sdi_during_turn_in_place = true;
        }

        m_safety_msg_mtx.lock();
        m_safety_msg.sdi_during_turn_in_place = sdi_during_turn_in_place;
        m_safety_msg_mtx.unlock();

        // If the PDS state is not OPERATION_ENABLED, we send a nil speed.
        if (!m_pds_ok) {
            p_left_speed = p_right_speed = 0;
        }

        // Send the actual speed (in RPM) to left motor
        err = m_left_controller.setTargetVelocity(static_cast<int16_t>(p_left_speed));
        if (ERROR_NONE != err) {
            RCLCPP_ERROR(get_logger(),
                         "Failed setting velocity of left motor, EZW_ERR: SMCService : "
                         "Controller::setTargetVelocity() return error code : %d",
                         (int)err);
            return;
        }

        // Send the actual speed (in RPM) to right motor
        err = m_right_controller.setTargetVelocity(static_cast<int16_t>(p_right_speed));
        if (ERROR_NONE != err) {
            RCLCPP_ERROR(get_logger(),
                         "Failed setting velocity of right motor, EZW_ERR: SMCService : "
                         "Controller::setTargetVelocity() return error code : %d",
                         (int)err);
            return;
        }

#if VERBOSE_OUTPUT
        RCLCPP_INFO(get_logger(), "Speed sent to motors (left, right) = (%d, %d) rpm", p_left_speed, p_right_speed);
#endif
    }

    void DiffDriveController::cbTimerSafety()
    {
        static bool m_first_entry = true;

        swd_ros2_controllers::msg::SafetyFunctions msg;
        ezw_error_t err;
        bool res_l, res_r;

        if (!m_nmt_ok) {
            return;
        }

        msg.header.stamp = get_clock()->now();
        msg.header.frame_id = m_params->getBaseFrame();

        // Reading SAFEIN_1
        ezw::smccore::ISafeMotionService::SafetyWordType safety_control_word;

        err = m_left_controller.getSafetyControlWord(ezw::smccore::ISafeMotionService::SafetyControlWordId::SAFEIN_1, safety_control_word);
        if (ERROR_NONE != err) {
            RCLCPP_ERROR(get_logger(),
                         "Error reading SAFEIN_1 control word from left motor, EZW_ERR: SMCService : "
                         "Controller::getSafetyControlWord() return error code : %d",
                         (int)err);
            return;
        }
        bool safein1_l[6];
        safein1_l[0] = safety_control_word.safety_function_0;
        safein1_l[1] = safety_control_word.safety_function_1;
        safein1_l[2] = safety_control_word.safety_function_2;
        safein1_l[3] = safety_control_word.safety_function_3;
        safein1_l[4] = safety_control_word.safety_function_4;
        safein1_l[5] = safety_control_word.safety_function_5;

        err = m_right_controller.getSafetyControlWord(ezw::smccore::ISafeMotionService::SafetyControlWordId::SAFEIN_1, safety_control_word);
        if (ERROR_NONE != err) {
            RCLCPP_ERROR(get_logger(),
                         "Error reading SAFEIN_1 control word from right motor, EZW_ERR: SMCService : "
                         "Controller::getSafetyControlWord() return error code : %d",
                         (int)err);
            return;
        }
        bool safein1_r[6];
        safein1_r[0] = safety_control_word.safety_function_0;
        safein1_r[1] = safety_control_word.safety_function_1;
        safein1_r[2] = safety_control_word.safety_function_2;
        safein1_r[3] = safety_control_word.safety_function_3;
        safein1_r[4] = safety_control_word.safety_function_4;
        safein1_r[5] = safety_control_word.safety_function_5;

        // Reading SBC
        res_l = m_left_safety_functions.count(ezw::smccore::ISafeMotionService::SafetyFunctionId::SBC_1) ? safein1_l[m_left_safety_functions[ezw::smccore::ISafeMotionService::SafetyFunctionId::SBC_1]] : true;
        res_r = m_right_safety_functions.count(ezw::smccore::ISafeMotionService::SafetyFunctionId::SBC_1) ? safein1_r[m_right_safety_functions[ezw::smccore::ISafeMotionService::SafetyFunctionId::SBC_1]] : true;

        msg.safe_brake_control = static_cast<uint8_t>(!res_l || !res_r);
        if (m_first_entry || msg.safe_brake_control != m_safety_msg.safe_brake_control) {
            RCLCPP_INFO(get_logger(), msg.safe_brake_control ? "SBC enabled." : "SBC disabled.");
        }

        // Reading STO
        res_l = m_left_safety_functions.count(ezw::smccore::ISafeMotionService::SafetyFunctionId::STO) ? safein1_l[m_left_safety_functions[ezw::smccore::ISafeMotionService::SafetyFunctionId::STO]] : true;
        res_r = m_right_safety_functions.count(ezw::smccore::ISafeMotionService::SafetyFunctionId::STO) ? safein1_r[m_right_safety_functions[ezw::smccore::ISafeMotionService::SafetyFunctionId::STO]] : true;

        msg.safe_torque_off = static_cast<uint8_t>(!res_l || !res_r);
        if (m_first_entry || msg.safe_torque_off != m_safety_msg.safe_torque_off) {
            RCLCPP_INFO(get_logger(), msg.safe_torque_off ? "STO enabled." : "STO disabled.");
        }

        // Reading SDI
        bool sdi_l_p, sdi_l_n, sdi_r_p, sdi_r_n, sdi_p, sdi_n;

        sdi_l_p = m_left_safety_functions.count(ezw::smccore::ISafeMotionService::SafetyFunctionId::SDIP_1) ? safein1_l[m_left_safety_functions[ezw::smccore::ISafeMotionService::SafetyFunctionId::SDIP_1]] : true;
        sdi_r_p = m_right_safety_functions.count(ezw::smccore::ISafeMotionService::SafetyFunctionId::SDIP_1) ? safein1_r[m_right_safety_functions[ezw::smccore::ISafeMotionService::SafetyFunctionId::SDIP_1]] : true;

        sdi_l_n = m_left_safety_functions.count(ezw::smccore::ISafeMotionService::SafetyFunctionId::SDIN_1) ? safein1_l[m_left_safety_functions[ezw::smccore::ISafeMotionService::SafetyFunctionId::SDIN_1]] : true;
        sdi_r_n = m_right_safety_functions.count(ezw::smccore::ISafeMotionService::SafetyFunctionId::SDIN_1) ? safein1_r[m_right_safety_functions[ezw::smccore::ISafeMotionService::SafetyFunctionId::SDIN_1]] : true;

        if (m_left_motor_polarity) {
            sdi_p = !sdi_l_n || !sdi_r_p;
            sdi_n = !sdi_l_p || !sdi_r_n;
        }
        else {
            sdi_p = !sdi_l_p || !sdi_r_n;
            sdi_n = !sdi_l_n || !sdi_r_p;
        }

        msg.safe_direction_indication_forward = static_cast<uint8_t>(sdi_p);
        msg.safe_direction_indication_backward = static_cast<uint8_t>(sdi_n);

        if (m_first_entry || msg.safe_direction_indication_forward != m_safety_msg.safe_direction_indication_forward) {
            RCLCPP_INFO(get_logger(), msg.safe_direction_indication_forward ? "SDIp enabled." : "SDIp disabled.");
        }

        if (m_first_entry || msg.safe_direction_indication_backward != m_safety_msg.safe_direction_indication_backward) {
            RCLCPP_INFO(get_logger(), msg.safe_direction_indication_backward ? "SDIn enabled." : "SDIn disabled.");
        }

        // Reading SLS_1
        // When an object is detected in the SLS area, res_r and res_l will be false.
        // If both or either res_l or res_r are false, set safety_limited_speed to true.
        res_l = m_left_safety_functions.count(ezw::smccore::ISafeMotionService::SafetyFunctionId::SLS_1) ? safein1_l[m_left_safety_functions[ezw::smccore::ISafeMotionService::SafetyFunctionId::SLS_1]] : true;
        res_r = m_right_safety_functions.count(ezw::smccore::ISafeMotionService::SafetyFunctionId::SLS_1) ? safein1_r[m_right_safety_functions[ezw::smccore::ISafeMotionService::SafetyFunctionId::SLS_1]] : true;

        msg.safety_limited_speed_1 = static_cast<uint8_t>(!res_l || !res_r);
        if (m_first_entry || msg.safety_limited_speed_1 != m_safety_msg.safety_limited_speed_1) {
            RCLCPP_INFO(get_logger(), msg.safety_limited_speed_1 ? "SLS_1 enabled." : "SLS_1 disabled.");
        }

        // Reading SLS_2
        res_l = m_left_safety_functions.count(ezw::smccore::ISafeMotionService::SafetyFunctionId::SLS_2) ? safein1_l[m_left_safety_functions[ezw::smccore::ISafeMotionService::SafetyFunctionId::SLS_2]] : true;
        res_r = m_right_safety_functions.count(ezw::smccore::ISafeMotionService::SafetyFunctionId::SLS_2) ? safein1_r[m_right_safety_functions[ezw::smccore::ISafeMotionService::SafetyFunctionId::SLS_2]] : true;

        msg.safety_limited_speed_2 = static_cast<uint8_t>(!res_l || !res_r);
        if (m_first_entry || msg.safety_limited_speed_2 != m_safety_msg.safety_limited_speed_2) {
            RCLCPP_INFO(get_logger(), msg.safety_limited_speed_2 ? "SLS_2 enabled." : "SLS_2 disabled.");
        }

#if VERBOSE_OUTPUT
        RCLCPP_INFO(get_logger(), "STO: %d, SDI+: %d, SDI-: %d, SLS_1: %d, SLS_2: %d", msg.safe_torque_off, msg.safe_direction_indication_forward, msg.safe_direction_indication_backward, msg.safety_limited_speed_1, msg.safety_limited_speed_2);
#endif

        m_safety_msg_mtx.lock();
        msg.sdi_during_turn_in_place = m_safety_msg.sdi_during_turn_in_place;
        m_safety_msg = msg;
        m_safety_msg_mtx.unlock();

        if (m_params->getPublishSafety()) {
            m_pub_safety->publish(msg);
        }

        m_first_entry = false;
    }

    void DiffDriveController::cbResetOdometry(const std_msgs::msg::Empty::SharedPtr p_reset_odometry)
    {
        (void)p_reset_odometry;

        // Acquire odom mutex
        std::lock_guard<std::mutex> lock(m_odom_msg_mtx);

        // Reset odometry position and orientation to zero
        m_x_prev = 0.0;
        m_y_prev = 0.0;
        m_theta_prev = 0.0;

        m_x_prev_err = 0.0;
        m_y_prev_err = 0.0;
        m_theta_prev_err = 0.0;

        RCLCPP_INFO(get_logger(), "Odometry reset to (0, 0, 0).");
    }

    void DiffDriveController::cbTimerWatchdogReceive()
    {
        // Stop the robot
        setSpeeds(0, 0);
    }

}  // namespace ezw::swd
