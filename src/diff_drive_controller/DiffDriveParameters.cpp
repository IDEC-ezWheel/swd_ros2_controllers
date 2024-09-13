
/**
 * Copyright (C) 2022 ez-Wheel S.A.S.
 *
 * @file DiffDriveParameters.cpp
 */

#include "diff_drive_controller/DiffDriveParameters.hpp"

namespace ezw::swd {
    DiffDriveParameters::DiffDriveParameters(rclcpp::Node *p_node) : m_node(p_node)
    {
        if (m_node == nullptr) {
            throw std::runtime_error("Valid node expected");
        }

        // Declare all parameters
        {
            rcl_interfaces::msg::ParameterDescriptor descriptor;
            descriptor.additional_constraints = "Positive value (0.0-999.0)";
            descriptor.type = rclcpp::ParameterType::PARAMETER_DOUBLE;
            descriptor.floating_point_range.resize(1);
            auto &range = descriptor.floating_point_range.at(0);
            range.from_value = 0.001;  // min value
            range.to_value = 999.0;    // max value
            range.step = 0.001;        // precision value

            m_node->declare_parameter<double>("baseline_m", DEFAULT_BASELINE_M, descriptor);
        }
        {
            rcl_interfaces::msg::ParameterDescriptor descriptor;
            descriptor.read_only = true;
            m_node->declare_parameter<std::string>("left_swd_config_file", DEFAULT_LEFT_CONFIG_FILE, descriptor);
            m_node->declare_parameter<std::string>("right_swd_config_file", DEFAULT_RIGHT_CONFIG_FILE, descriptor);
        }
        {
            rcl_interfaces::msg::ParameterDescriptor descriptor;
            descriptor.additional_constraints = "Positive value (0-999)";
            descriptor.type = rclcpp::ParameterType::PARAMETER_INTEGER;
            descriptor.integer_range.resize(1);
            auto &range = descriptor.integer_range.at(0);
            range.from_value = 1;  // min value
            range.to_value = 999;  // max value
            range.step = 1;        // precision value

            m_node->declare_parameter<int>("pub_freq_hz", DEFAULT_PUB_FREQ_HZ, descriptor);
        }
        {
            rcl_interfaces::msg::ParameterDescriptor descriptor;
            descriptor.additional_constraints = "Positive value (0-9999)";
            descriptor.type = rclcpp::ParameterType::PARAMETER_INTEGER;
            descriptor.integer_range.resize(1);
            auto &range = descriptor.integer_range.at(0);
            range.from_value = 1;   // min value
            range.to_value = 9999;  // max value
            range.step = 1;         // precision value
            descriptor.read_only = true;
            m_node->declare_parameter<int>("watchdog_receive_ms", DEFAULT_WATCHDOG_RECEIVE_MS, descriptor);
        }
        m_node->declare_parameter<std::string>("base_frame", DEFAULT_BASE_FRAME);
        m_node->declare_parameter<std::string>("odom_frame", DEFAULT_ODOM_FRAME);
        m_node->declare_parameter<bool>("publish_odom", DEFAULT_PUBLISH_ODOM);
        m_node->declare_parameter<bool>("publish_tf", DEFAULT_PUBLISH_TF);
        m_node->declare_parameter<bool>("publish_safety_functions", DEFAULT_PUBLISH_SAFETY_FCNS);
        {
            rcl_interfaces::msg::ParameterDescriptor descriptor;
            descriptor.additional_constraints = "Positive value (50-2000)";
            descriptor.type = rclcpp::ParameterType::PARAMETER_INTEGER;
            descriptor.integer_range.resize(1);
            auto &range = descriptor.integer_range.at(0);
            range.from_value = 50;  // min value
            range.to_value = 2000;  // max value
            range.step = 1;         // precision value

            m_node->declare_parameter<int>("motor_max_speed_rpm", DEFAULT_MOTOR_MAX_SPEED_RPM, descriptor);
            m_node->declare_parameter<int>("motor_max_safety_limited_speed_1_rpm", DEFAULT_MOTOR_MAX_SLS_1_RPM, descriptor);
            m_node->declare_parameter<int>("motor_max_safety_limited_speed_2_rpm", DEFAULT_MOTOR_MAX_SLS_2_RPM, descriptor);
            m_node->declare_parameter<int>("motor_max_delta_speed_rpm", DEFAULT_MOTOR_MAX_DELTA_RPM, descriptor);
        }
        m_node->declare_parameter<bool>("have_backward_sls", DEFAULT_HAVE_BACKWARD_SLS);
        {
            rcl_interfaces::msg::ParameterDescriptor descriptor;
            descriptor.additional_constraints = "Percent value (0.0-1.0)";
            descriptor.type = rclcpp::ParameterType::PARAMETER_DOUBLE;
            descriptor.floating_point_range.resize(1);
            auto &range = descriptor.floating_point_range.at(0);
            range.from_value = 0.01;  // min value
            range.to_value = 1.0;     // max value
            range.step = 0.01;        // precision value

            m_node->declare_parameter<double>("left_encoder_relative_error", DEFAULT_LEFT_RELATIVE_ERROR, descriptor);
            m_node->declare_parameter<double>("right_encoder_relative_error", DEFAULT_RIGHT_RELATIVE_ERROR, descriptor);
        }
        m_node->declare_parameter<bool>("accurate_odometry", DEFAULT_ACCURATE_ODOMETRY);
        m_node->declare_parameter<bool>("publish_wheel_odometry", DEFAULT_PUBLISH_WHEEL_ODOMETRY);
    }

    auto DiffDriveParameters::getBaseline() -> double
    {
        // Baseline, distance between wheels (m)
        auto l_baseline_m = m_baseline_m;
        m_node->get_parameter("baseline_m", m_baseline_m);
        if (m_baseline_m != l_baseline_m) {
            RCLCPP_INFO(m_node->get_logger(), "baseline_m : %f", m_baseline_m);
        }
        return m_baseline_m;
    }

    auto DiffDriveParameters::getLeftConfigFile() -> std::string
    {
        // Left config file
        auto l_left_config_file = m_left_config_file;
        m_node->get_parameter("left_swd_config_file", m_left_config_file);
        if (m_left_config_file != l_left_config_file) {
            RCLCPP_INFO(m_node->get_logger(), "left_swd_config_file : %s", m_left_config_file.c_str());
        }
        return m_left_config_file;
    }

    auto DiffDriveParameters::getRightConfigFile() -> std::string
    {
        // Right config file
        auto l_right_config_file = m_right_config_file;
        m_node->get_parameter("right_swd_config_file", m_right_config_file);
        if (m_right_config_file != l_right_config_file) {
            RCLCPP_INFO(m_node->get_logger(), "right_swd_config_file : %s", m_right_config_file.c_str());
        }
        return m_right_config_file;
    }

    auto DiffDriveParameters::getPubFreqHz() -> int
    {
        // Pub freq (Hz)
        auto l_pub_freq_hz = m_pub_freq_hz;
        m_node->get_parameter("pub_freq_hz", m_pub_freq_hz);
        if (m_pub_freq_hz != l_pub_freq_hz) {
            RCLCPP_INFO(m_node->get_logger(), "pub_freq_hz : %d", m_pub_freq_hz);
        }
        return m_pub_freq_hz;
    }

    auto DiffDriveParameters::getWatchdogReceiveMs() -> int
    {
        // Watchdog receive (ms)
        auto l_watchdog_receive_ms = m_watchdog_receive_ms;
        m_node->get_parameter("watchdog_receive_ms", m_watchdog_receive_ms);
        if (m_watchdog_receive_ms != l_watchdog_receive_ms) {
            RCLCPP_INFO(m_node->get_logger(), "watchdog_receive_ms : %d", m_watchdog_receive_ms);
        }
        return m_watchdog_receive_ms;
    }

    auto DiffDriveParameters::getBaseFrame() -> std::string
    {
        // Base frame
        auto l_base_frame = m_base_frame;
        m_node->get_parameter("base_frame", m_base_frame);
        if (m_base_frame != l_base_frame) {
            RCLCPP_INFO(m_node->get_logger(), "base_frame : %s", m_base_frame.c_str());
        }
        return m_base_frame;
    }

    auto DiffDriveParameters::getOdomFrame() -> std::string
    {
        // Odom frame
        auto l_odom_frame = m_odom_frame;
        m_node->get_parameter("odom_frame", m_odom_frame);
        if (m_odom_frame != l_odom_frame) {
            RCLCPP_INFO(m_node->get_logger(), "odom_frame : %s", m_odom_frame.c_str());
        }
        return m_odom_frame;
    }

    auto DiffDriveParameters::getPublishOdom() -> bool
    {
        // Publish odom
        auto l_publish_odom = m_publish_odom;
        m_node->get_parameter("publish_odom", m_publish_odom);
        if (m_publish_odom != l_publish_odom) {
            RCLCPP_INFO(m_node->get_logger(), "publish_odom : %d", m_publish_odom);
        }
        return m_publish_odom;
    }

    auto DiffDriveParameters::getPublishTf() -> bool
    {
        // Publish tf
        auto l_publish_tf = m_publish_tf;
        m_node->get_parameter("publish_tf", m_publish_tf);
        if (m_publish_tf != l_publish_tf) {
            RCLCPP_INFO(m_node->get_logger(), "publish_tf : %d", m_publish_tf);
        }
        return m_publish_tf;
    }

    auto DiffDriveParameters::getPublishSafety() -> bool
    {
        // Publish safety
        auto l_publish_safety = m_publish_safety;
        m_node->get_parameter("publish_safety_functions", m_publish_safety);
        if (m_publish_safety != l_publish_safety) {
            RCLCPP_INFO(m_node->get_logger(), "publish_safety_functions : %d", m_publish_safety);
        }
        return m_publish_safety;
    }

    auto DiffDriveParameters::getHaveBackwardSls() -> bool
    {
        // Have backward SLS
        auto l_have_backward_sls = m_have_backward_sls;
        m_node->get_parameter("have_backward_sls", m_have_backward_sls);
        if (m_have_backward_sls != l_have_backward_sls) {
            RCLCPP_INFO(m_node->get_logger(), "have_backward_sls : %d", m_have_backward_sls);
        }
        return m_have_backward_sls;
    }

    auto DiffDriveParameters::getLeftEncoderRelativeError() -> float
    {
        // Left encoder relative error
        auto l_left_encoder_relative_error = m_left_encoder_relative_error;
        m_node->get_parameter("left_encoder_relative_error", m_left_encoder_relative_error);
        if (m_left_encoder_relative_error != l_left_encoder_relative_error) {
            RCLCPP_INFO(m_node->get_logger(), "left_encoder_relative_error : %f", m_left_encoder_relative_error);
        }
        return m_left_encoder_relative_error;
    }

    auto DiffDriveParameters::getRightEncoderRelativeError() -> float
    {
        // Right encoder relative error
        auto l_right_encoder_relative_error = m_right_encoder_relative_error;
        m_node->get_parameter("right_encoder_relative_error", m_right_encoder_relative_error);
        if (m_right_encoder_relative_error != l_right_encoder_relative_error) {
            RCLCPP_INFO(m_node->get_logger(), "right_encoder_relative_error : %f", m_right_encoder_relative_error);
        }
        return m_right_encoder_relative_error;
    }

    auto DiffDriveParameters::getMotorMaxSpeedRpm() -> int
    {
        // Motor max speed rpm
        auto l_motor_max_speed_rpm = m_motor_max_speed_rpm;
        m_node->get_parameter("motor_max_speed_rpm", m_motor_max_speed_rpm);
        if (m_motor_max_speed_rpm != l_motor_max_speed_rpm) {
            RCLCPP_INFO(m_node->get_logger(), "motor_max_speed_rpm : %d", m_motor_max_speed_rpm);
        }
        return m_motor_max_speed_rpm;
    }

    auto DiffDriveParameters::getMotorMaxSls1SpeedRpm() -> int
    {
        // Motor max speed rpm when SLS_1 enabled
        auto l_motor_max_sls_1_speed_rpm = m_motor_max_sls_1_speed_rpm;
        m_node->get_parameter("motor_max_safety_limited_speed_1_rpm", m_motor_max_sls_1_speed_rpm);
        if (m_motor_max_sls_1_speed_rpm != l_motor_max_sls_1_speed_rpm) {
            RCLCPP_INFO(m_node->get_logger(), "motor_max_safety_limited_speed_1_rpm : %d", m_motor_max_sls_1_speed_rpm);
        }
        return m_motor_max_sls_1_speed_rpm;
    }

    auto DiffDriveParameters::getMotorMaxSls2SpeedRpm() -> int
    {
        // Motor max speed rpm when SLS_2 enabled
        auto l_motor_max_sls_2_speed_rpm = m_motor_max_sls_2_speed_rpm;
        m_node->get_parameter("motor_max_safety_limited_speed_2_rpm", m_motor_max_sls_2_speed_rpm);
        if (m_motor_max_sls_2_speed_rpm != l_motor_max_sls_2_speed_rpm) {
            RCLCPP_INFO(m_node->get_logger(), "motor_max_safety_limited_speed_2_rpm : %d", m_motor_max_sls_2_speed_rpm);
        }
        return m_motor_max_sls_2_speed_rpm;
    }

    auto DiffDriveParameters::getMotorMaxDeltaSpeedRpm() -> int
    {
        // Motor max delta speed rpm between the both motors
        auto l_motor_max_delta_speed_rpm = m_motor_max_delta_speed_rpm;
        m_node->get_parameter("motor_max_delta_speed_rpm", m_motor_max_delta_speed_rpm);
        if (m_motor_max_delta_speed_rpm != l_motor_max_delta_speed_rpm) {
            RCLCPP_INFO(m_node->get_logger(), "motor_max_delta_speed_rpm : %d", m_motor_max_delta_speed_rpm);
        }
        return m_motor_max_delta_speed_rpm;
    }

    auto DiffDriveParameters::getAccurateOdometry() -> bool
    {
        // Accurate odometry
        auto l_accurate_odometry = m_accurate_odometry;
        m_node->get_parameter("accurate_odometry", m_accurate_odometry);
        if (m_accurate_odometry != l_accurate_odometry) {
            RCLCPP_INFO(m_node->get_logger(), "accurate_odometry : %d", m_accurate_odometry);
        }
        return m_accurate_odometry;
    }

    auto DiffDriveParameters::getPublishWheelOdometry() -> bool
    {
        // Publish Wheel odometry
        auto l_publish_wheel_odometry = m_publish_wheel_odometry;
        m_node->get_parameter("publish_wheel_odometry", m_publish_wheel_odometry);
        if (m_publish_wheel_odometry != l_publish_wheel_odometry) {
            RCLCPP_INFO(m_node->get_logger(), "publish_wheel_odometry : %d", m_publish_wheel_odometry);
        }
        return m_publish_wheel_odometry;
    }

}  // namespace ezw::swd
